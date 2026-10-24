"""只读 TX 逐帧事件审计：与独立线路抓包按 VL/FIFO 次序交叉关联。"""
import argparse
import bisect
import csv
import hashlib
import json
import tempfile
from collections import defaultdict
from pathlib import Path


def require(ok, message):
    if not ok:
        raise ValueError(message)


def rows(path):
    with path.open(newline="", encoding="utf-8") as source:
        return [{k: int(v) for k, v in row.items()} for row in csv.DictReader(source)]


def audit(directory, scenario):
    prefix = directory / f"load_{scenario}"
    sources = {suffix: Path(f"{prefix}{suffix}") for suffix in
               ("_tx.csv", "_echo.csv", "_tx_events.txt")}
    messages = defaultdict(list)
    for r in rows(sources["_tx.csv"]):
        messages[r["vl"]].append(dict(kind="background", id=r["id"], vl=r["vl"],
            payload_bytes=r["payload_bytes"], commit_ns=r["commit_ns"],
            out_a_sfd_ns=r["out_a_sfd_ns"], out_b_sfd_ns=r["out_b_sfd_ns"]))
    for r in rows(sources["_echo.csv"]):
        messages[1].append(dict(kind="echo", id=r["id"], vl=1,
            payload_bytes=r["payload_bytes"], commit_ns=r["tx_commit_ns"],
            out_a_sfd_ns=r["out_a_sfd_ns"], out_b_sfd_ns=r["out_b_sfd_ns"]))
    events = defaultdict(lambda: defaultdict(list))
    masks = []
    previous_time = -1
    with sources["_tx_events.txt"].open(encoding="utf-8") as source:
        for line in source:
            fields = line.split()
            require(fields[0] in ("M", "Q", "G", "P", "H", "E", "A", "B", "a", "b"),
                    "unknown profile event")
            require(int(fields[1]) >= previous_time, "profile event time regressed")
            previous_time = int(fields[1])
            if fields[0] == "M":
                masks.append((int(fields[1]), int(fields[2], 16)))
            else:
                events[int(fields[2])][fields[0]].append(tuple(map(int, fields[1:])))
    require(masks, "missing scheduler mask timeline")
    mask_times = [m[0] for m in masks]
    names = ["wire_eligible", "window", "credit", "mac_b", "mac_a",
             "scheduler_bag", "nonempty"]
    categories = ["same_vl_ahead_ns", "mac_bag_ns", "scheduler_bag_ns",
                  "wire_credit_ns", "window_ns", "config_hold_ns",
                  "queue_publish_ns", "builder_busy_ns", "arbitration_ns"]
    result = []
    for vl, group in messages.items():
        group.sort(key=lambda r: r["commit_ns"])
        e = events[vl]
        for stage in "QGPHEABab":
            require(len(e[stage]) == len(group), f"VL{vl} stage {stage} count mismatch")
        previous_pop = 0
        for index, message in enumerate(group):
            stamps = {stage: e[stage][index][0] for stage in "QGPHEABab"}
            require(e["Q"][index][2] == message["payload_bytes"], f"VL{vl} queue length mismatch")
            require(message["commit_ns"] <= stamps["Q"] <= stamps["G"] < stamps["H"] < stamps["E"] <= stamps["P"],
                    f"VL{vl} queue/build causality failure")
            for start, end, out in (("A", "a", "out_a_sfd_ns"), ("B", "b", "out_b_sfd_ns")):
                require(stamps["H"] <= stamps[start] < stamps[end] < message[out],
                        f"VL{vl} MAC/capture causality failure")
            counts = dict.fromkeys(categories, 0)
            begin, finish = stamps["Q"], stamps["G"]
            head = max(begin, min(finish, previous_pop))
            counts["same_vl_ahead_ns"] = head - begin
            cursor = head
            mask_index = bisect.bisect_right(mask_times, cursor) - 1
            require(mask_index >= 0, "queue predates mask timeline")
            while cursor < finish:
                value = masks[mask_index][1]
                bits = {name: bool((value >> (2 + 128 * i + vl)) & 1)
                        for i, name in enumerate(names)}
                stop = min(finish, masks[mask_index + 1][0] if mask_index + 1 < len(masks) else finish)
                if not (bits["mac_a"] and bits["mac_b"]):
                    reason = "mac_bag_ns"
                elif not bits["scheduler_bag"]:
                    reason = "scheduler_bag_ns"
                elif not bits["credit"]:
                    reason = "wire_credit_ns"
                elif not bits["window"]:
                    reason = "window_ns"
                elif value & 1:
                    reason = "config_hold_ns"
                elif not bits["nonempty"]:
                    reason = "queue_publish_ns"
                elif not (value & 2):
                    reason = "builder_busy_ns"
                else:
                    reason = "arbitration_ns"
                counts[reason] += stop - cursor
                cursor = stop
                mask_index += 1
            require(sum(counts.values()) == finish - begin, "wait categories do not partition interval")
            # 完全由真实握手/状态边沿计算；MAC 项含整帧提交后的同步与线路排队。
            row = dict(message, queue_commit_ns=begin, grant_ns=finish,
                header_first_ns=stamps["H"], builder_last_ns=stamps["E"],
                mac_a_first_ns=stamps["A"], mac_b_first_ns=stamps["B"],
                mac_a_last_ns=stamps["a"], mac_b_last_ns=stamps["b"], **counts,
                admission_pipeline_ns=begin-message["commit_ns"],
                queue_wait_ns=finish-begin, setup_ns=stamps["H"]-finish,
                assembly_ns=stamps["E"]-stamps["H"],
                mac_a_tail_ns=message["out_a_sfd_ns"]-stamps["E"],
                mac_b_tail_ns=message["out_b_sfd_ns"]-stamps["E"],
                commit_to_sfd_ns=max(message["out_a_sfd_ns"],message["out_b_sfd_ns"])-message["commit_ns"])
            require(row["commit_to_sfd_ns"] == row["admission_pipeline_ns"] + row["queue_wait_ns"] +
                    row["setup_ns"] + row["assembly_ns"] + max(row["mac_a_tail_ns"],row["mac_b_tail_ns"]),
                    "end-to-end TX partition mismatch")
            result.append(row)
            previous_pop = stamps["P"]
    require(result, "empty TX profile")
    require(set(events) == set(messages), "unmatched profile VL")
    result.sort(key=lambda r: r["commit_ns"])
    with Path(f"{prefix}_tx_profile.csv").open("w", newline="", encoding="utf-8") as target:
        writer = csv.DictWriter(target, fieldnames=list(result[0]))
        writer.writeheader()
        writer.writerows(result)
    fields = categories + ["queue_wait_ns", "setup_ns", "assembly_ns", "mac_a_tail_ns", "commit_to_sfd_ns"]
    summary = dict(scenario=scenario, messages=len(result),
        captures_sha256={k: hashlib.sha256(p.read_bytes()).hexdigest() for k, p in sources.items()},
        maxima={field: max(r[field] for r in result) for field in fields},
        echo=[r for r in result if r["kind"] == "echo"],
        worst=max(result, key=lambda r: r["commit_to_sfd_ns"]))
    Path(f"{prefix}_tx_profile.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return summary


def self_test():
    # 最小独立夹具：错序、漏事件、长度错、丢时间线及不相干 VL 必须被拒绝。
    artifact_dir = Path(__file__).resolve().parents[2] / "2-sim"
    with tempfile.TemporaryDirectory(prefix="tx_profile_selftest_", dir=artifact_dir) as folder:
        directory = Path(folder)
        prefix = directory / "load_0"
        Path(f"{prefix}_tx.csv").write_text("id,vl\n", encoding="utf-8")
        Path(f"{prefix}_echo.csv").write_text(
            "id,payload_bytes,tx_commit_ns,out_a_sfd_ns,out_b_sfd_ns\n0,17,100,960,960\n", encoding="utf-8")
        mask = ((((1 << (7*128)) - 1) << 2) | 2)
        original = (f"M 120 {mask:x}\nQ 124 1 17\nG 200 1\nH 304 1\nA 352 1\nB 352 1\n"
                    "E 840 1\nP 856 1\na 856 1\nb 856 1\n")
        capture = Path(f"{prefix}_tx_events.txt")
        capture.write_text(original, encoding="utf-8")
        good = audit(directory, 0)
        require(good["messages"] == 1 and good["echo"][0]["queue_wait_ns"] == 76,
                "self-test valid fixture failed")
        variants = [original.replace("Q 124 1 17", "Q 124 1 18"),
                    original.replace("G 200 1\n", ""),
                    original.replace("G 200 1", "G 100 1"),
                    "\n".join(original.splitlines()[1:])+"\n",
                    original.replace("a 856 1", "a 856 2"),
                    original+"Z 900 1\n"]
        for variant in variants:
            capture.write_text(variant, encoding="utf-8")
            try:
                audit(directory, 0)
            except ValueError:
                pass
            else:
                raise ValueError("self-test corrupt fixture was accepted")
    print("TX_PROFILE_SELF_TEST_PASS valid=1 rejected=6")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=int, choices=range(5))
    parser.add_argument("--directory", type=Path, default=Path("."))
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
    if args.scenario is not None:
        summary = audit(args.directory, args.scenario)
        print(f"TX_PROFILE_PASS scenario={args.scenario} messages={summary['messages']} maxima={summary['maxima']}")
    elif not args.self_test:
        parser.error("--scenario or --self-test is required")


if __name__ == "__main__":
    main()
