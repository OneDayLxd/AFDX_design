"""Audit two independent endpoints, valid-only application ports and A/B links.

October 26 FAQ: first application input byte to the corresponding first received
byte; jitter is grouped by VL AND frame length. Legacy first-to-last/mixed-length
50us guards remain separate diagnostics. October 30 C139.2 adds a strict <100us
full-load jitter check; raw same-VL/same-length range is retained without BAG subtraction.
VLID header bytes are not payload.
"""
import argparse
import copy
import hashlib
import json
from collections import defaultdict
from pathlib import Path

from frame_reference import parse_frame, payload, require


def read_capture(path):
    records = defaultdict(list)
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        require(parts and parts[0] in ("CONFIG", "O", "A", "W", "I", "N", "D", "END"), "capture tag")
        tag = parts[0]
        if tag in ("W", "I", "D", "A"):
            records[tag].append(tuple(map(int, parts[1:-1])) + (bytes.fromhex(parts[-1]),))
        else:
            records[tag].append(tuple(map(int, parts[1:])))
    return records


def unique(rows, key, label):
    result = {}
    for row in rows:
        k = key(row)
        require(k not in result, f"duplicate {label}: {k}")
        result[k] = row
    return result


def audit(r):
    require(len(r["CONFIG"]) == len(r["END"]) == 1, "completion/config marker")
    scenario, nvl, rounds, period, extra = r["CONFIG"][0]
    require(scenario in range(7), "scenario")
    require((nvl, rounds, period, extra) ==
            (127 if scenario in (1, 6) else 20 if scenario == 5 else 8,
             8 if scenario in (2, 6) else 12 if scenario == 4 else 4,
             1471*80 if scenario == 5 else 2_000_000 if scenario == 6 else 2_500_000,
             5000 if scenario == 3 else 0), "load contract")
    count = nvl * rounds
    require(r["END"][0] == (count, count, 0), "completion counts/errors")
    offers = unique(r["O"], lambda v: v[:2], "application offer")
    actual_inputs = unique(r["A"], lambda v: v[:2], "actual application input")
    delivered = unique(r["D"], lambda v: (1-v[0], v[1]), "application delivery")
    keys = {(src, i) for src in range(2) for i in range(count)}
    require(set(offers) == set(actual_inputs) == set(delivered) == keys, "missing/extra application message")
    for key, (src, i, vlid, first, payload_first, last, raw) in actual_inputs.items():
        _, _, vl, length, offered, commit = offers[key]
        require(vlid == 0x1000+vl and raw[:2] == vlid.to_bytes(2, "big"), "input actual VLID")
        require(raw[2:] == payload(i, vl, length), "actual input payload")
        require((first, last) == (offered, commit) and first < payload_first <= last,
                "actual input timestamps")
        if scenario == 5:
            require(payload_first-first == 16 and last-payload_first == (length-1)*80,
                    "100Mbps payload pacing within each message")
    plans = unique(r["N"], lambda v: v[:3], "network plan")
    require(set(plans) == {(src, net, i) for src, i in keys for net in range(2)}, "network plan completeness")
    for (src, net, i), (_, _, _, action, delay) in plans.items():
        expected_action = 0
        expected_delay = extra
        if scenario == 2:
            if (i % 6 == 1 and net == 1) or (i % 6 == 2 and net == 0):
                expected_action = 1
            if (i % 6 == 3 and net == 0) or (i % 6 == 4 and net == 1):
                expected_action = 2
        require((action, delay) == (expected_action, expected_delay), "network fault/delay contract")
    outgoing, incoming = {}, {}
    for tag, target in (("W", outgoing), ("I", incoming)):
        prior_eof, prior_sfd, prior_sn = {}, {}, {}
        for endpoint, net, sfd, eof, raw in r[tag]:
            require(endpoint in (0, 1) and net in (0, 1), "wire endpoint/network")
            src = endpoint if tag == "W" else 1-endpoint
            i = int.from_bytes(raw[50:54], "big")
            key = src, net, i
            require(key in plans and key not in target, "unexpected/duplicate physical frame")
            action = plans[key][3]
            info = parse_frame(raw, sfd, eof, tag == "W" or action != 2)
            require(info[0] == i and info[3] in range(1, 256), "identity/nonzero sequence")
            require((src, i) in offers and info[1:3] == offers[src, i][2:4], "wire/source VL and length")
            lane = endpoint, net
            require(lane not in prior_eof or sfd-prior_eof[lane] >= 160, "IFG >=12 byte clocks")
            prior_eof[lane] = eof
            if tag == "W":
                vk = endpoint, net, info[1]
                require(vk not in prior_sfd or sfd-prior_sfd[vk] >= 2_000_000, "BAG >=2ms")
                require(info[3] == prior_sn.get(vk, 0)+1, "per-VL sequence")
                prior_sfd[vk], prior_sn[vk] = sfd, info[3]
            target[key] = sfd, eof, raw
    require(set(outgoing) == set(plans), "every message has two TX copies")
    require(set(incoming) == {k for k, v in plans.items() if v[3] != 1}, "link output completeness")
    for k, (sfd, eof, raw) in incoming.items():
        tx_sfd, tx_eof, original = outgoing[k]
        expected = bytearray(original)
        if plans[k][3] == 2:
            expected[58] ^= 1
        require(raw == expected, "link changed undeclared bytes")
        require(sfd-tx_sfd == eof-tx_eof == plans[k][4], "direct GMII timestamps and declared TB delay")
    timing, directions, jitter_groups = [], [], []
    for src in range(2):
        per_vl = defaultdict(list)
        fixed_length = defaultdict(list)
        first_byte_latencies = []
        first_times, last_times, total_bytes = [], [], 0
        prior_id = {}
        # Delivery order is not necessarily VL order. Check each VL independently.
        rows = sorted((v for k, v in delivered.items() if k[0] == src), key=lambda v: v[5])
        for dst, i, vl, length, first, last, raw in rows:
            _, _, source_vl, source_len, offered, commit = offers[src, i]
            round_id = i//nvl
            expected_len = 1471 if scenario in (1, 5, 6) else (17, 465, 977, 1471)[
                (round_id//3 if scenario == 4 else round_id) % 4]
            require((vl, length) == (source_vl, source_len) == (i % nvl+1, expected_len), "message metadata")
            require(raw == (0x1000+vl).to_bytes(2, "big")+payload(i, vl, length), "application VLID/payload")
            require(i > prior_id.get(vl, -1), "application same-VL order")
            prior_id[vl] = i
            tx = min(outgoing[src, net, i][0] for net in range(2))
            good = [incoming[src, net, i][0] for net in range(2)
                    if plans[src, net, i][3] == 0]
            require(good, "no good redundant copy")
            rx = min(good)
            require(offered <= commit <= tx <= rx <= first <= last, "timestamp causality")
            require(last-first >= (length-1)*8, "application byte duration")
            latency = last-offered
            first_byte_latency = first-offered
            first_byte_latencies.append(first_byte_latency)
            frame_bytes = len(outgoing[src, 0, i][2])-8  # DA..FCS，去掉前导码/SFD。
            require(frame_bytes == len(outgoing[src, 1, i][2])-8, "A/B MAC frame lengths")
            fixed_length[vl, frame_bytes].append((first_byte_latency, i))
            per_vl[vl].append(latency)
            first_times.append(offered)
            last_times.append(last)
            total_bytes += length
            timing.append(dict(source=src, id=i, vl=vl, payload_bytes=length,
                               application_latency_ns=latency, post_commit_tx_ns=tx-commit,
                               first_byte_latency_ns=first_byte_latency,
                               earliest_tx_sfd_ns=tx, earliest_good_rx_sfd_ns=rx,
                               application_first_ns=first, application_last_ns=last))
        window = max(last_times)-min(first_times)
        groups = []
        for (vl, frame_bytes), samples in sorted(fixed_length.items()):
            group = dict(source=src, vl=vl, mac_frame_bytes=frame_bytes, samples=len(samples),
                         jitter_us=(max(samples)[0]-min(samples)[0])/1000 if len(samples)>1 else None)
            groups.append(group)
        jitter_groups.extend(groups)
        repeated = [g["jitter_us"] for g in groups if g["samples"]>1]
        input_first = min(v[4] for k, v in actual_inputs.items() if k[0] == src)
        input_last = max(v[5] for k, v in actual_inputs.items() if k[0] == src)
        # 包含最后一个载荷字节的完整采样间隔；首部不计有效载荷。
        input_window = input_last-input_first+(80 if scenario == 5 else 8)
        input_mbps = total_bytes*8000/input_window
        if scenario == 5:
            times = sorted(v[3] for k, v in actual_inputs.items() if k[0] == src)
            require(all(b-a == 1471*80 for a, b in zip(times, times[1:])), "continuous input rate contract")
            require(input_window == total_bytes*80, "measured 100Mbps input window")
        directions.append(dict(source=src, messages=count, payload_bytes=total_bytes,
                               window_ns=window, payload_mbps=total_bytes*8000/window,
                               input_window_ns=input_window, input_payload_mbps=input_mbps,
                               first_byte_max_latency_us=max(first_byte_latencies)/1000,
                               first_byte_latency_below_1ms=max(first_byte_latencies) < 1_000_000,
                               same_length_jitter_us=max(repeated) if repeated else None,
                               repeated_jitter_groups=len(repeated), total_jitter_groups=len(groups),
                               app_max_latency_us=max(max(v) for v in per_vl.values())/1000,
                               app_max_same_vl_jitter_us=max(max(v)-min(v) for v in per_vl.values())/1000))
    guards = dict(application_latency_2ms=all(d["app_max_latency_us"] <= 2000 for d in directions),
                  application_jitter_50us=all(d["app_max_same_vl_jitter_us"] <= 50 for d in directions))
    if scenario in (1, 6):
        guards["payload_100mbps_each_direction"] = all(d["payload_mbps"] >= 100 for d in directions)
    formal = dict(first_byte_latency_below_2ms=all(d["first_byte_max_latency_us"] < 2000 for d in directions))
    if scenario in (1, 2, 4, 5, 6):
        formal["repeated_same_vl_same_length_samples"] = all(
            d["repeated_jitter_groups"] == d["total_jitter_groups"] for d in directions)
    if scenario == 6:
        # 所有业务 VL 在每个 BAG 内提供 Lmax；检查实际激励，不仅相信 CONFIG。
        for src in range(2):
            for vl in range(1, nvl+1):
                times = [offers[src, round_id*nvl+vl-1][4] for round_id in range(rounds)]
                require(all(b-a == 2_000_000 for a,b in zip(times,times[1:])),
                        "every active VL must be offered exactly at configured BAG")
        formal["all_127_business_vls_at_lmax_each_bag"] = True
    if scenario == 5:
        formal["continuous_input_payload_100mbps"] = all(d["input_payload_mbps"] >= 100 for d in directions)
    if all(d["same_length_jitter_us"] is not None for d in directions):
        formal["same_length_jitter_below_100us"] = all(d["same_length_jitter_us"] < 100 for d in directions)
        guards["same_length_jitter_50us"] = all(d["same_length_jitter_us"] <= 50 for d in directions)
    return dict(scenario=scenario, functional_audit="PASS", nvl=nvl, rounds=rounds,
                production_capacity="128 TX / 4096 RX; VL0 reserved for management",
                model="two independent endpoints, A/B direct GMII, zero added latency; optional TB faults/delay",
                metric_scope="FAQ C119/C120: VLID+payload first input byte to corresponding first output byte",
                jitter_limit_us=100, jitter_limit_note="FAQ C139.2: strictly below 100us under declared load; raw max-minus-min, no BAG subtraction; legacy 50us guard retained",
                directions=directions, acceptance_checks=formal, engineering_guards=guards,
                jitter_groups=jitter_groups, messages=timing)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", type=int, choices=range(7))
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument("--compare-delay", action="store_true")
    args = parser.parse_args()
    if args.compare_delay:
        baseline = read_capture(Path("two_endpoints_0.txt"))
        delayed = read_capture(Path("two_endpoints_3.txt"))
        audit(baseline)
        audit(delayed)
        base_tx = unique(baseline["W"], lambda v: v[:2]+(int.from_bytes(v[-1][50:54], "big"),), "base TX")
        delayed_tx = unique(delayed["W"], lambda v: v[:2]+(int.from_bytes(v[-1][50:54], "big"),), "delayed TX")
        require(base_tx == delayed_tx, "delay control changed sender traffic")
        base_rx = unique(baseline["I"], lambda v: v[:2]+(int.from_bytes(v[-1][50:54], "big"),), "base RX")
        delayed_rx = unique(delayed["I"], lambda v: v[:2]+(int.from_bytes(v[-1][50:54], "big"),), "delayed RX")
        require(set(base_rx) == set(delayed_rx), "delay control missing RX")
        for k, a in base_rx.items():
            b = delayed_rx[k]
            require(a[-1] == b[-1] and b[2]-a[2] == b[3]-a[3] == 5000,
                    "every GMII frame must gain exactly 5us link delay")
        print("Delay control PASS: all RX SFD/EOF timestamps shifted by exactly 5000ns; sender unchanged.")
        return 0
    if args.scenario is None:
        parser.error("--scenario or --compare-delay is required")
    path = Path(f"two_endpoints_{args.scenario}.txt")
    records = read_capture(path)
    result = audit(records)
    require(result["scenario"] == args.scenario, "capture scenario mismatch")
    if args.selftest:
        for mutation in ("payload", "missing_copy", "link_delay", "input_vlid", "input_time"):
            broken = copy.deepcopy(records)
            if mutation == "payload":
                v = broken["D"][0]
                b = bytearray(v[-1]); b[-1] ^= 1
                broken["D"][0] = v[:-1] + (bytes(b),)
            elif mutation == "missing_copy":
                broken["W"].pop()
            elif mutation == "link_delay":
                v = broken["N"][0]
                broken["N"][0] = v[:-1] + (v[-1]+8,)
            elif mutation == "input_vlid":
                v = broken["A"][0]
                broken["A"][0] = v[:2] + (v[2]^1,) + v[3:]
            else:
                v = broken["A"][0]
                broken["A"][0] = v[:3] + (v[3]+8,) + v[4:]
            try:
                audit(broken)
            except ValueError:
                continue
            raise ValueError(f"negative control was not rejected: {mutation}")
        result["negative_controls"] = "5/5 rejected"
    result["capture_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    Path(f"two_endpoints_{args.scenario}_summary.json").write_text(
        json.dumps(result, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k not in ("messages", "jitter_groups")}, indent=2))
    return 0 if all(result["engineering_guards"].values()) and all(result["acceptance_checks"].values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
