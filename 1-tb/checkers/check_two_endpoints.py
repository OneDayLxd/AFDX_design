"""Audit two independent endpoints, valid-only application ports and A/B links.

Application latency guards are engineering checks, not a replacement for the
organizer's GMII SFD boundary. Network time is reported separately.
"""
import argparse
import copy
import hashlib
import json
from collections import defaultdict
from pathlib import Path

from check_app_loopback import parse_frame, payload, require


def read_capture(path):
    records = defaultdict(list)
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        require(parts and parts[0] in ("CONFIG", "O", "W", "I", "N", "D", "END"), "capture tag")
        tag = parts[0]
        if tag in ("W", "I", "D"):
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
    require(scenario in range(4), "scenario")
    require((nvl, rounds, period, extra) ==
            (127 if scenario == 1 else 8, 8 if scenario == 2 else 4,
             2_500_000, 5000 if scenario == 3 else 0), "load contract")
    count = nvl * rounds
    require(r["END"][0] == (count, count, 0), "completion counts/errors")
    offers = unique(r["O"], lambda v: v[:2], "application offer")
    delivered = unique(r["D"], lambda v: (1-v[0], v[1]), "application delivery")
    keys = {(src, i) for src in range(2) for i in range(count)}
    require(set(offers) == set(delivered) == keys, "missing/extra application message")
    plans = unique(r["N"], lambda v: v[:3], "network plan")
    require(set(plans) == {(src, net, i) for src, i in keys for net in range(2)}, "network plan completeness")
    for (src, net, i), (_, _, _, action, delay) in plans.items():
        expected_action = 0
        expected_delay = (256 if net == 0 else 512) + extra
        if scenario == 2:
            if (i % 6 == 1 and net == 1) or (i % 6 == 2 and net == 0):
                expected_action = 1
            if (i % 6 == 3 and net == 0) or (i % 6 == 4 and net == 1):
                expected_action = 2
            if i % 6 == 5:
                expected_delay = 1024 if net == 0 else 256
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
        require(sfd-56 >= tx_eof+plans[k][4], "store/forward delay and causality")
    timing, directions = [], []
    for src in range(2):
        per_vl = defaultdict(list)
        first_times, last_times, total_bytes = [], [], 0
        prior_id = {}
        # Delivery order is not necessarily VL order. Check each VL independently.
        rows = sorted((v for k, v in delivered.items() if k[0] == src), key=lambda v: v[5])
        for dst, i, vl, length, first, last, raw in rows:
            _, _, source_vl, source_len, offered, commit = offers[src, i]
            expected_len = 1471 if scenario == 1 else (17, 465, 977, 1471)[(i//nvl) % 4]
            require((vl, length) == (source_vl, source_len) == (i % nvl+1, expected_len), "message metadata")
            require(raw == payload(i, vl, length), "application payload")
            require(i > prior_id.get(vl, -1), "application same-VL order")
            prior_id[vl] = i
            tx = min(outgoing[src, net, i][0] for net in range(2))
            good = [incoming[src, net, i][0] for net in range(2)
                    if plans[src, net, i][3] == 0]
            require(good, "no good redundant copy")
            rx = min(good)
            require(offered <= commit <= tx < rx <= first <= last, "timestamp causality")
            require(last-first >= (length-1)*8, "application byte duration")
            latency = last-offered
            per_vl[vl].append(latency)
            first_times.append(offered)
            last_times.append(last)
            total_bytes += length
            timing.append(dict(source=src, id=i, vl=vl, payload_bytes=length,
                               application_latency_ns=latency, post_commit_tx_ns=tx-commit,
                               earliest_tx_sfd_ns=tx, earliest_good_rx_sfd_ns=rx,
                               application_first_ns=first, application_last_ns=last))
        window = max(last_times)-min(first_times)
        directions.append(dict(source=src, messages=count, payload_bytes=total_bytes,
                               window_ns=window, payload_mbps=total_bytes*8000/window,
                               app_max_latency_us=max(max(v) for v in per_vl.values())/1000,
                               app_max_same_vl_jitter_us=max(max(v)-min(v) for v in per_vl.values())/1000))
    guards = dict(application_latency_2ms=all(d["app_max_latency_us"] <= 2000 for d in directions),
                  application_jitter_50us=all(d["app_max_same_vl_jitter_us"] <= 50 for d in directions))
    if scenario == 1:
        guards["payload_100mbps_each_direction"] = all(d["payload_mbps"] >= 100 for d in directions)
    return dict(scenario=scenario, functional_audit="PASS", nvl=nvl, rounds=rounds,
                production_capacity="128 TX / 4096 RX; VL0 reserved for management",
                model="two independent endpoints, A/B store-and-forward, 256/512ns base delay",
                metric_scope="application-to-application engineering guards; not official GMII processing signoff",
                directions=directions, engineering_guards=guards, messages=timing)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", type=int, choices=range(4))
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
        for mutation in ("payload", "missing_copy", "link_delay"):
            broken = copy.deepcopy(records)
            if mutation == "payload":
                v = broken["D"][0]
                b = bytearray(v[-1]); b[-1] ^= 1
                broken["D"][0] = v[:-1] + (bytes(b),)
            elif mutation == "missing_copy":
                broken["W"].pop()
            else:
                v = broken["N"][0]
                broken["N"][0] = v[:-1] + (v[-1]+8,)
            try:
                audit(broken)
            except ValueError:
                continue
            raise ValueError(f"negative control was not rejected: {mutation}")
        result["negative_controls"] = "3/3 rejected"
    result["capture_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    Path(f"two_endpoints_{args.scenario}_summary.json").write_text(
        json.dumps(result, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "messages"}, indent=2))
    return 0 if all(result["engineering_guards"].values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
