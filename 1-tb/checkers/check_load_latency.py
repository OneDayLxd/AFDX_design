"""Independently audit load-test CSV timing/correlation; never waive RTL guards."""
import argparse
import csv
import json
import re
from collections import defaultdict
from pathlib import Path


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return [{k: int(v) for k, v in row.items()} for row in csv.DictReader(stream)]


def metrics(log, tag):
    lines = re.findall(r"^# " + tag + r" (.*)$", log, re.MULTILINE)
    require(len(lines) == 1, f"{tag}: missing or duplicated completion record")
    return {k: int(v) for k, v in re.findall(r"(\w+)=(-?\d+)", lines[0])}


def group_jitter(rows, key, value):
    groups = defaultdict(list)
    for row in rows:
        groups[row[key]].append(row[value])
    repeated = [(max(values) - min(values), vl) for vl, values in groups.items() if len(values) > 1]
    return max(repeated, default=(0, -1))


def audit(directory, scenario):
    prefix = directory / f"load_{scenario}"
    log = prefix.with_suffix(".log").read_text(encoding="utf-8", errors="replace")
    result = metrics(log, "LOAD_RESULT")
    counts = metrics(log, "LOAD_COUNTS")
    drops = metrics(log, "LOAD_DROPS")
    jitters = metrics(log, "LOAD_JITTER")
    complete = metrics(log, "LOAD_COMPLETE")
    echo = read_csv(Path(f"{prefix}_echo.csv"))
    rx = read_csv(Path(f"{prefix}_rx.csv"))
    tx = read_csv(Path(f"{prefix}_tx.csv"))
    inputs = read_csv(Path(f"{prefix}_input.csv"))
    require(all(row["scenario"] == scenario for rows in (echo, rx, tx, inputs) for row in rows),
            "mixed scenario capture files")
    require(complete["scenario"] == scenario and complete["functional_errors"] == 0,
            "RTL functional checker failed; timing results are not accepted")
    require(len(inputs) == counts["bg_rx"] + 16, "not all released RX stimuli reached GMII")
    source = {(r["vl"], r["id"]): r for r in inputs}
    require(len(source) == len(inputs), "duplicate injected sample ID")
    previous = {}
    input_min_gap = None
    for row in inputs:
        key = row["vl"]
        if key in previous:
            gap = row["input_sfd_ns"] - previous[key]
            require(gap >= 2_000_000, "RX stimulus violates per-VL BAG")
            input_min_gap = gap if input_min_gap is None else min(gap, input_min_gap)
        previous[key] = row["input_sfd_ns"]
    delivered = {}
    last_rx_id = {}
    for row in rx:
        key = row["vl"], row["id"]
        require(key in source and key not in delivered, f"RX missing source/duplicate: {key}")
        require(row["id"] > last_rx_id.get(row["vl"], -1), "RX per-VL order")
        last_rx_id[row["vl"]] = row["id"]
        require(row["input_sfd_ns"] == source[key]["input_sfd_ns"], "RX source timestamp mismatch")
        require(row["payload_bytes"] == source[key]["payload_bytes"], "RX source length mismatch")
        require(row["input_sfd_ns"] <= row["app_first_ns"] <= row["app_last_ns"], "RX causality")
        require(row["rx_ns"] == row["app_last_ns"] - row["input_sfd_ns"], "RX duration")
        delivered[key] = row
    require(len(rx) == counts["rx_delivered"], "RX row count mismatch")
    missing = sorted(set(source) - set(delivered))
    require(len(missing) == counts["missing_rx"], "missing RX conservation")
    require(sum(vl == 1 for vl, _ in missing) == counts["missing_target"], "missing targets")
    require(len(echo) == result["echo_n"], "echo sample count mismatch")
    require(len({r["id"] for r in echo}) == len(echo), "duplicate echo ID")
    echo_values, same_length = [], defaultdict(list)
    for row in echo:
        key = 1, row["id"]
        require(key in delivered, "response without application delivery")
        require(row["rx_last_ns"] == delivered[key]["app_last_ns"], "echo RX association")
        stamps = [row[k] for k in ("input_sfd_ns", "rx_first_ns", "rx_last_ns",
                                  "tx_offer_ns", "tx_accept_ns", "tx_commit_ns")]
        require(stamps == sorted(stamps) and min(row["out_a_sfd_ns"], row["out_b_sfd_ns"]) >= stamps[-1],
                "echo causality")
        duration = min(row["out_a_sfd_ns"], row["out_b_sfd_ns"]) - row["input_sfd_ns"]
        require(duration == row["echo_ns"], "echo duration")
        echo_values.append(duration)
        same_length[row["payload_bytes"]].append(duration)
    require(echo_values, "no surviving target samples")
    require(min(echo_values) == result["echo_min_ns"] and max(echo_values) == result["echo_max_ns"],
            "echo min/max mismatch")
    require(max(echo_values) - min(echo_values) == result["echo_jitter_ns"], "echo jitter mismatch")
    fixed = max(max(v) - min(v) for v in same_length.values())
    require(fixed == result["fixed_jitter_ns"], "fixed-length jitter mismatch")
    require(len(tx) + counts["missing_tx"] == counts["bg_tx"], "TX background conservation")
    for row in tx:
        stamps = [row[k] for k in ("release_ns", "offer_ns", "accept_ns", "commit_ns")]
        require(stamps == sorted(stamps), "TX causality")
        require(min(row["out_a_sfd_ns"], row["out_b_sfd_ns"]) >= stamps[-1], "TX before commit")
        require(row["tx_ns"] == min(row["out_a_sfd_ns"], row["out_b_sfd_ns"]) - row["accept_ns"],
                "TX duration")
    require(len({r["id"] for r in tx}) == len(tx), "duplicate TX ID")
    commit_order = defaultdict(list)
    for row in tx:
        commit_order[row["vl"]].append((row["commit_ns"], row["out_a_sfd_ns"], row["out_b_sfd_ns"]))
    for row in echo:
        commit_order[1].append((row["tx_commit_ns"], row["out_a_sfd_ns"], row["out_b_sfd_ns"]))
    for values in commit_order.values():
        values.sort()
        for earlier, later in zip(values, values[1:]):
            require(later[1] - earlier[1] >= 2_000_000 and later[2] - earlier[2] >= 2_000_000,
                    "TX per-VL FIFO order/BAG")
    rx_jitter, _ = group_jitter(rx, "vl", "rx_ns")
    tx_jitter, _ = group_jitter(tx, "vl", "tx_ns")
    require(rx_jitter == jitters["rx_same_vl_max_ns"], "RX per-VL jitter mismatch")
    require(tx_jitter == jitters["tx_same_vl_max_ns"], "TX per-VL jitter mismatch")
    require(max(r["rx_ns"] for r in rx) == result["rx_max_ns"], "RX max mismatch")
    tx_delays = [r["tx_ns"] for r in tx] + [
        min(r["out_a_sfd_ns"], r["out_b_sfd_ns"]) - r["tx_accept_ns"] for r in echo]
    require(max(tx_delays) == result["tx_accept_to_sfd_max_ns"], "TX max mismatch")
    require(sum(d >= 2_000_000 for d in echo_values) == counts["echo_over_2ms"], "echo deadline count")
    accounted = len(rx) + drops["rm"] + drops["ic"] + drops["rx_full"] + drops["mac_ovf"]
    require(accounted == 2 * len(inputs), "A/B physical-frame drop/delivery conservation")
    # 仅分解已观测 TX 等待；剩余部分还包含组帧/MAC 等待，不能全归因于仲裁。
    # 使用实际上一帧 SFD，包含场景 4 排在回送前面的同 VL 背景消息。
    vl1_outputs = sorted(
        [min(r["out_a_sfd_ns"], r["out_b_sfd_ns"]) for r in tx if r["vl"] == 1]
        + [min(r["out_a_sfd_ns"], r["out_b_sfd_ns"]) for r in echo])
    previous_output = {later: earlier for earlier, later in zip(vl1_outputs, vl1_outputs[1:])}
    wait_breakdown = []
    for row in echo:
        output = min(row["out_a_sfd_ns"], row["out_b_sfd_ns"])
        commit = row["tx_commit_ns"]
        bag_due = previous_output[output] + 2_000_000 if output in previous_output else commit
        bag_wait = max(0, bag_due - commit)
        other_wait = output - commit - bag_wait
        require(other_wait >= 0, "negative TX residual after actual-SFD BAG bound")
        wait_breakdown.append({
            "id": row["id"], "commit_to_sfd_ns": output - commit,
            "prior_frame_bag_lower_bound_ns": bag_wait, "remaining_tx_wait_ns": other_wait})
    summary = {
        **result, **counts, **drops, **jitters,
        "functional_errors": complete["functional_errors"],
        "guard_errors": complete["total_guard_errors"],
        "capture_audit": "PASS",
        "input_samples": len(inputs),
        "rx_vl_covered": len({r["vl"] for r in rx}),
        "tx_vl_covered": len({r["vl"] for r in tx} | ({1} if echo else set())),
        "input_min_same_vl_gap_ns": input_min_gap,
        "tx_source_queue_max_ns": max((r["offer_ns"] - r["release_ns"] for r in tx), default=0),
        "tx_admission_max_ns": max([r["accept_ns"] - r["offer_ns"] for r in tx] +
                                  [r["tx_accept_ns"] - r["tx_offer_ns"] for r in echo]),
        "tx_commit_to_sfd_max_ns": max(
            [min(r["out_a_sfd_ns"], r["out_b_sfd_ns"]) - r["commit_ns"] for r in tx] +
            [min(r["out_a_sfd_ns"], r["out_b_sfd_ns"]) - r["tx_commit_ns"] for r in echo]),
        "tx_message_backpressure_max_ns": max(
            [r["commit_ns"] - r["accept_ns"] - 8 * (r["payload_bytes"] - 1) for r in tx] +
            [r["tx_commit_ns"] - r["tx_accept_ns"] - 8 * (r["payload_bytes"] - 1) for r in echo]),
        "echo_same_length": {str(length): {"n": len(values), "min_ns": min(values),
                              "max_ns": max(values), "jitter_ns": max(values) - min(values)}
                             for length, values in sorted(same_length.items())},
        "missing_rx_ids": missing,
        "echo_tx_wait_breakdown": wait_breakdown,
    }
    Path(f"{prefix}_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2),
                                            encoding="utf-8")
    print(f"LOAD_AUDIT scenario={scenario} PASS; guard_errors={summary['guard_errors']}; "
          f"echo_max_us={max(echo_values)/1000:.3f}; "
          f"jitter_us={(max(echo_values)-min(echo_values))/1000:.3f}; missing_rx={len(missing)}")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=Path("."))
    parser.add_argument("--scenario", type=int, required=True)
    args = parser.parse_args()
    audit(args.directory, args.scenario)
