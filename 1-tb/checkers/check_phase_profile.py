"""Independent CSV audit and conditional bound calculation for the phase profile."""
import argparse
import csv
import json
import re
from pathlib import Path


def audit(path, phased):
    log = path.with_suffix(".log").read_text(encoding="utf-8", errors="replace")
    records = re.findall(r"^# PHASE_RESULT (.*)$", log, re.MULTILINE)
    assert len(records) == 1, "missing/duplicate simulation completion"
    measured = dict(re.findall(r"(\w+)=([\d.]+)", records[0]))
    assert re.search(r"tb_afdx_phase_profile: \d+ checks, 0 errors -> PASS", log), "RTL scoreboard did not pass"
    assert int(measured["enable"]) == int(phased)
    assert int(measured["rx_unique"]) == 4992 and int(measured["samples"]) == 1024
    with path.open(newline="", encoding="utf-8") as stream:
        rows = [{key: int(value) for key, value in row.items()} for row in csv.DictReader(stream)]
    assert len(rows) == 1024, f"expected 1024 logical samples, got {len(rows)}"
    identity = {(row["vl"], row["round"]) for row in rows}
    assert identity == {(v, r) for v in range(128) for r in range(8)}, "missing/duplicate VL sample"
    by_vl = {v: [] for v in range(128)}
    for row in rows:
        v, r = row["vl"], row["round"]
        size = [17, 465, 977, 1471, 1471, 1471, 1471, 1471][r]
        if v == 0:
            size = min(size, 516)
        assert row["payload_bytes"] == size
        source, app, commit = (row[k] for k in ("input_sfd_ns", "app_last_ns", "commit_ns"))
        a, b, slot = (row[k] for k in ("out_a_sfd_ns", "out_b_sfd_ns", "slot_ns"))
        assert slot <= source <= slot + 8100, "source phase contract violated"
        assert source < app < commit < min(a, b), "noncausal echo correlation"
        assert commit < slot + 59000, "application commit deadline violated"
        assert row["echo_ns"] == a - source
        if phased:
            assert 50000 <= min(a, b) - source <= max(a, b) - source <= 80000
            assert slot + 60000 <= min(a, b) <= max(a, b) <= slot + 74048, "launch bound violated"
        by_vl[v].append(row)
    jitter = []
    for samples in by_vl.values():
        samples.sort(key=lambda row: row["round"])
        for net in ("out_a_sfd_ns", "out_b_sfd_ns"):
            assert all(b[net] - a[net] >= 2000000 for a, b in zip(samples, samples[1:])), "wire BAG"
        delays = [min(row["out_a_sfd_ns"], row["out_b_sfd_ns"]) - row["input_sfd_ns"] for row in samples]
        jitter.append(max(delays) - min(delays))
    payload = sum(row["payload_bytes"] for row in rows)
    throughput = payload * 8000 / (8 * 8192000)
    assert int(measured["payload_bytes"]) == payload
    assert abs(float(measured["throughput_mbps"]) - throughput) < 0.000001
    assert int(measured["max_jitter_ns"]) == max(jitter), "TB/CSV jitter disagreement"
    assert throughput >= 100, "payload throughput <100Mbps"
    if phased:
        assert max(jitter) <= 50000
    # Conditional deterministic bound: queue ready before 59us, only its own
    # slot may start a frame, no pending older frame, both MACs available.
    # Reserve the entire 1us window, 32 reset/phase cycles, 128 control cycles
    # and the longest streamed payload; these exceed the observed RTL paths.
    upper_ns = (7500 + 125 + 32 + 128 + 1471) * 8
    lower_ns = 7500 * 8 - 8100
    assert upper_ns < 80000 and lower_ns > 50000
    assert upper_ns - lower_ns < 50000
    # Even if a maximum frame starts at the conservative upper bound, it
    # ends well before the next slot's grant; thus no wire blocking accumulates.
    assert upper_ns + 1518 * 8 < 64000 + 7500 * 8
    result = {
        "status": "PROFILE_PASS" if phased else "RR_CONTROL_FUNCTIONAL_PASS",
        "logical_samples": len(rows), "tx_vl": 128, "rx_vl_tested_by_tb": 4096,
        "unique_payload_bytes": payload, "measurement_interval_ns": 8 * 8192000,
        "payload_mbps": throughput,
        "min_echo_ns": min(row["echo_ns"] for row in rows),
        "max_echo_ns": max(row["echo_ns"] for row in rows),
        "max_same_vl_jitter_ns": max(jitter),
        "max_data_vl_jitter_ns": max(jitter[1:]),
        "max_data_vl_echo_ns": max(row["echo_ns"] for row in rows if row["vl"] != 0),
        "vl0_jitter_ns": jitter[0],
        "max_rx_ns": max(row["app_last_ns"] - row["input_sfd_ns"] for row in rows),
        "max_commit_offset_ns": max(row["commit_ns"] - row["slot_ns"] for row in rows),
        "rx_peak_a": int(measured["rx_peak_a"]), "rx_peak_b": int(measured["rx_peak_b"]),
        "conditional_bound_ns": [lower_ns, upper_ns] if phased else None,
        "conditional_jitter_bound_ns": upper_ns - lower_ns if phased else None,
        "scope": "conditional on phase, burst, commit deadline and idle egress; not an unrestricted AFDX guarantee",
    }
    path.with_name(path.stem + "_summary.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--control", action="store_true")
    args = parser.parse_args()
    audit(args.csv, not args.control)
