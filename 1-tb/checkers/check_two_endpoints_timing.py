"""Decompose existing two-endpoint traffic using passive internal handshakes.

Each message uses the actual RX-committed network, not an assumed earliest copy.
All intervals telescope to the original application latency. This is diagnostic,
not a change to the organizer's acceptance definition or the old 50us guard.
"""
import argparse
import copy
import csv
import hashlib
import json
from collections import defaultdict
from pathlib import Path

from check_app_loopback import require
from check_two_endpoints import audit, read_capture, unique


STAGES = (
    "input_collect", "ingress_wait", "ingress_read", "tx_handoff",
    "tx_queue_wait", "build_and_mac", "network_sfd", "rx_wire_frame",
    "rx_pre_copy", "rx_pool_copy_commit", "rx_delivery_wait", "app_output",
)


def load_trace(path):
    result = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        tag, *values = line.split()
        require(tag in ("H", "Q", "G", "R") and len(values) == 7, "trace schema")
        endpoint, i, vl, first, last, net, length = map(int, values)
        src = 1-endpoint if tag == "R" else endpoint
        key = src, i, tag
        require(key not in result, "duplicate trace event")
        result[key] = dict(vl=vl, first=first, last=last, net=net, length=length)
    return result


def decompose(records, trace):
    baseline = audit(records)  # Revalidate payload, CRC, BAG, ordering and faults.
    offers = unique(records["O"], lambda r: r[:2], "offer")
    deliveries = unique(records["D"], lambda r: (1-r[0], r[1]), "delivery")
    def wire_key(r):
        return r[0], r[1], int.from_bytes(r[-1][50:54], "big")
    outgoing = unique(records["W"], wire_key, "TX")
    incoming = unique(records["I"], wire_key, "RX")
    plans = unique(records["N"], lambda r: r[:3], "plan")
    require(set(trace) == {(s, i, t) for s, i in offers for t in "HQGR"},
            "missing/extra stage events")
    rows = []
    for (src, i), (_, _, vl, length, offered, commit) in sorted(offers.items()):
        h, q, g, r = (trace[src, i, t] for t in "HQGR")
        require(all(t["vl"] == vl for t in (h, q, g, r)), "stage VL/ID mismatch")
        require(all(t["length"] == length for t in (h, q, r)), "stage payload length")
        net = r["net"]
        require(net in (0, 1) and plans[src, net, i][3] == 0, "selected bad/missing copy")
        tx = outgoing[src, net, i]
        rx = incoming[1-src, net, i]
        delivery = deliveries[src, i]
        points = (offered, commit, h["first"], h["last"], q["last"],
                  g["first"], tx[2], rx[2], rx[3], r["first"], r["last"],
                  delivery[4], delivery[5])
        require(all(a <= b for a, b in zip(points, points[1:])), "stage causality")
        require(h["first"] <= q["first"] <= q["last"], "TX pipeline ordering")
        durations = dict(zip(STAGES, (b-a for a, b in zip(points, points[1:]))))
        require(sum(durations.values()) == delivery[5]-offered, "latency sum")
        row = dict(source=src, id=i, vl=vl, length=length, selected_net=net,
                   latency_ns=delivery[5]-offered, durations_ns=durations,
                   times_ns=dict(zip(("offer", "input_last", "ingress_first", "ingress_last",
                                      "tx_queue_last", "grant", "tx_sfd", "rx_sfd", "rx_eof",
                                      "rx_copy_first", "rx_commit", "app_first", "app_last"), points)))
        rows.append(row)
    pairs = []
    groups = defaultdict(list)
    for row in rows:
        groups[row["source"], row["vl"]].append(row)
    for (src, vl), group in groups.items():
        fastest = min(group, key=lambda r: r["latency_ns"])
        slowest = max(group, key=lambda r: r["latency_ns"])
        delta = {s: slowest["durations_ns"][s]-fastest["durations_ns"][s] for s in STAGES}
        jitter = slowest["latency_ns"]-fastest["latency_ns"]
        require(sum(delta.values()) == jitter, "jitter sum")
        pairs.append(dict(source=src, vl=vl, jitter_ns=jitter, fastest_id=fastest["id"],
                          slowest_id=slowest["id"], delta_ns=delta))
    worst = [max((p for p in pairs if p["source"] == src), key=lambda p: p["jitter_ns"])
             for src in (0, 1)]
    require(all(p["jitter_ns"]/1000 == baseline["directions"][p["source"]]["app_max_same_vl_jitter_us"]
                for p in worst), "baseline jitter agreement")
    return dict(scenario=baseline["scenario"], diagnostic_audit="PASS", messages=rows,
                per_vl_pairs=pairs, worst_pairs=worst)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True, type=int, choices=range(4))
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    capture = Path(f"two_endpoints_{args.scenario}.txt")
    trace_path = Path(f"two_endpoints_{args.scenario}_stages.txt")
    records = read_capture(capture)
    trace = load_trace(trace_path)
    result = decompose(records, trace)
    if args.selftest:
        for mutation in ("missing", "time", "network"):
            broken = copy.deepcopy(trace)
            key = next(k for k in broken if k[2] == "R")
            if mutation == "missing":
                del broken[key]
            elif mutation == "time":
                broken[key]["last"] = 0
            else:
                broken[key]["net"] = 2
            try:
                decompose(records, broken)
            except ValueError:
                continue
            raise ValueError(f"negative control not rejected: {mutation}")
        result["negative_controls"] = "3/3 rejected"
    result["capture_sha256"] = hashlib.sha256(capture.read_bytes()).hexdigest()
    result["trace_sha256"] = hashlib.sha256(trace_path.read_bytes()).hexdigest()
    Path(f"two_endpoints_{args.scenario}_timing.json").write_text(
        json.dumps(result, indent=2)+"\n", encoding="utf-8")
    with Path(f"two_endpoints_{args.scenario}_timing.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=("source", "id", "vl", "length", "selected_net", "latency_ns")+STAGES)
        writer.writeheader()
        for row in result["messages"]:
            writer.writerow({**{k: row[k] for k in writer.fieldnames if k in row}, **row["durations_ns"]})
    print(json.dumps({k: v for k, v in result.items() if k not in ("messages", "per_vl_pairs")}, indent=2))


if __name__ == "__main__":
    main()
