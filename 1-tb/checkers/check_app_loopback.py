"""Audit application -> GMII network loop -> application, without application echo.

The 2 ms / 50 us checks are explicitly engineering guards on this topology.
They are NOT a claim about the organizer's ambiguous GMII SFD-to-SFD path.
"""
import argparse
import binascii
import copy
import csv
import hashlib
import json
from collections import defaultdict
from pathlib import Path


def require(condition, message):
    if not condition:
        raise ValueError(message)


def folded_sum(data):
    if len(data) % 2:
        data += b"\0"
    value = sum(int.from_bytes(data[i:i+2], "big") for i in range(0, len(data), 2))
    while value >> 16:
        value = (value & 65535) + (value >> 16)
    return value


def payload(msg_id, vl, length):
    return (msg_id.to_bytes(4, "big") + vl.to_bytes(2, "big") + length.to_bytes(2, "big")
            + bytes((msg_id*17 + vl + p*7) & 255 for p in range(8, length)))


def parse_frame(raw, sfd, eof, crc_good=True):
    require(raw[:8] == b"\x55"*7 + b"\xd5", "wire preamble/SFD")
    frame = raw[8:]
    require(64 <= len(frame) <= 1518, "Ethernet length")
    require(eof-sfd == len(frame)*8, "SFD to last-byte timing")
    crc_matches = binascii.crc32(frame[:-4]) == int.from_bytes(frame[-4:], "little")
    require(crc_matches == crc_good, "FCS good/bad expectation")
    require(frame[:4] == b"\x03\x00\x00\x00" and frame[12:15] == b"\x08\x00\x45", "AFDX/IPv4 header")
    require(frame[23] == 17 and folded_sum(frame[14:34]) == 65535, "IPv4 checksum/protocol")
    vl = int.from_bytes(frame[4:6], "big") - 0x1000
    length = int.from_bytes(frame[38:40], "big")-8
    require(length in (17,465,977,1471) and len(frame) == length+47, "payload/body length")
    require(int.from_bytes(frame[16:18], "big") == length+28, "IP total length")
    require(int.from_bytes(frame[34:36], "big") == 1000+vl and
            int.from_bytes(frame[36:38], "big") == 2000+vl, "UDP endpoints")
    body = frame[42:42+length]
    msg_id = int.from_bytes(body[:4], "big")
    if crc_good:
        pseudo = frame[26:34] + b"\0\x11" + frame[38:40]
        require(frame[40:42] != b"\0\0" and folded_sum(pseudo+frame[34:42+length]) == 65535,
                "independent UDP checksum")
        require(body == payload(msg_id, vl, length), "wire payload identity")
    return msg_id, vl, length, frame[-5]


def read_capture(path):
    records = defaultdict(list)
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        require(parts and parts[0] in ("CONFIG","O","F","C","W","N","I","D","END"), "unknown capture row")
        tag = parts[0]
        if tag in ("W", "I", "D"):
            records[tag].append(tuple(map(int, parts[1:-1])) + (bytes.fromhex(parts[-1]),))
        else:
            records[tag].append(tuple(map(int, parts[1:])))
    return records


def unique(rows, key, name):
    result = {}
    for row in rows:
        k = key(row)
        require(k not in result, f"duplicate {name}: {k}")
        result[k] = row
    return result


def distribution(values):
    require(values, "no complete timing samples")
    values = sorted(values)
    return dict(min_ns=values[0], max_ns=values[-1],
                p99_ns=values[(len(values)*99+99)//100-1])


def audit(records):
    require(len(records["CONFIG"]) == len(records["END"]) == 1, "missing/duplicate completion/config")
    scenario, n_vl, rounds, period, extra = records["CONFIG"][0]
    require(scenario in range(6), "scenario range")
    require((n_vl, rounds, period, extra) ==
            (127 if scenario in (1,5) else 8, 4 if scenario in (1,5) else 8,
             0 if scenario == 5 else 2_500_000, 5000 if scenario == 4 else 0), "scenario configuration")
    count = n_vl*rounds
    offered = unique(records["O"], lambda r:r[0], "offered ID")
    first = unique(records["F"], lambda r:r[0], "first accepted ID")
    commit = unique(records["C"], lambda r:r[0], "commit ID")
    delivery = unique(records["D"], lambda r:r[0], "application delivery")
    require(set(offered) == set(first) == set(commit) == set(delivery) == set(range(count)),
            "missing/extra application message or handshake")
    plans = unique(records["N"], lambda r:(r[0],r[1]), "network plan")
    require(set(plans) == {(n,i) for n in (0,1) for i in offered}, "network plan completeness")
    for (net, msg_id), (_, _, action, delay) in plans.items():
        mode = msg_id%6
        expected_action = 0
        expected_delay = (256 if net == 0 else 512)+extra
        if scenario == 2:
            if (mode == 1 and net == 1) or (mode == 2 and net == 0):
                expected_action = 1
            if (mode == 3 and net == 0) or (mode == 4 and net == 1):
                expected_action = 2
            if mode == 5:
                expected_delay = 1024 if net == 0 else 256
        require((action, delay) == (expected_action,expected_delay), "network fault/delay contract")
    outgoing, incoming = {}, {}
    for tag, target in (("W",outgoing),("I",incoming)):
        prior_eof, prior_vl_sfd, prior_sn = {}, {}, {}
        for net, sfd, eof, raw in records[tag]:
            require(net in (0,1), "wire network")
            msg_id = int.from_bytes(raw[50:54], "big")
            require((net,msg_id) in plans, "wire message without source")
            action = plans[net,msg_id][2]
            info = parse_frame(raw,sfd,eof, tag == "W" or action != 2)
            require(msg_id == info[0], "wire ID decode")
            key = net, msg_id
            require(key not in target, "duplicate physical frame")
            target[key] = (sfd,eof,raw,info)
            require(net not in prior_eof or sfd-prior_eof[net] >= 20*8, "wire IFG")
            prior_eof[net] = eof
            if tag == "W":
                vl_key = net, info[1]
                require(vl_key not in prior_vl_sfd or sfd-prior_vl_sfd[vl_key] >= 2_000_000, "TX BAG")
                require(info[3] == prior_sn.get(vl_key,0)+1, "TX per-VL sequence order")
                prior_vl_sfd[vl_key], prior_sn[vl_key] = sfd, info[3]
    require(set(outgoing) == set(plans), "missing A/B transmitted frame")
    require(set(incoming) == {k for k,p in plans.items() if p[2] != 1}, "missing/extra network output")
    for key, (sfd,eof,raw,_) in incoming.items():
        tx_sfd, tx_eof, original, _ = outgoing[key]
        action, delay = plans[key][2:]
        expected_raw = bytearray(original)
        if action == 2:
            expected_raw[58] ^= 1
        require(raw == expected_raw, "network changes beyond declared fault")
        # 首个前导码在 SFD 前 56ns；必须在原帧末字节和固定延迟之后。
        require(sfd-56 >= tx_eof+delay, "network forwarded before complete frame + delay")
    messages, e2e_per_vl, fixed_per_vl, lengths = [], defaultdict(list), defaultdict(list), defaultdict(set)
    rx_order = {}
    for row in records["D"]:
        msg_id, vl, length, sn, net, app_first, app_last, raw = row
        _, source_vl, source_len, offer = offered[msg_id]
        require((vl,length) == (source_vl,source_len), "application source metadata")
        require(raw == payload(msg_id,vl,length), "application payload mismatch")
        require(net in (0,1) and (net,msg_id) in incoming and plans[net,msg_id][2] != 2,
                "application selected a missing/bad copy")
        tx_sfd, tx_eof, _, tx_info = outgoing[net,msg_id]
        rx_sfd, rx_eof, _, _ = incoming[net,msg_id]
        require(sn == tx_info[3], "application sequence metadata")
        require(msg_id > rx_order.get(vl,-1), "application per-VL order")
        rx_order[vl] = msg_id
        accept, complete = first[msg_id][1], commit[msg_id][1]
        require(offer <= accept <= complete <= tx_sfd < tx_eof < rx_sfd < rx_eof <= app_first <= app_last,
                "stage timestamp causality")
        require(app_last-app_first >= (length-1)*8, "application byte timing")
        lengths[vl].add(length+47)
        e2e = app_last-accept
        e2e_per_vl[vl].append(e2e)
        fixed_per_vl[vl,length].append(e2e)
        stages = dict(admission_wait_ns=accept-offer, app_ingest_ns=complete-accept,
                      post_commit_tx_ns=tx_sfd-complete, tx_processing_ns=tx_sfd-accept,
                      network_sfd_ns=rx_sfd-tx_sfd, rx_processing_ns=app_last-rx_sfd,
                      rx_after_last_byte_ns=app_last-rx_eof, total_ns=e2e,
                      offer_to_delivery_ns=app_last-offer)
        require(e2e == stages["tx_processing_ns"]+stages["network_sfd_ns"]+stages["rx_processing_ns"],
                "stage duration conservation")
        messages.append(dict(id=msg_id,vl=vl,payload_bytes=length,selected_net=net,offer_ns=offer,
                             accept_ns=accept,commit_ns=complete,tx_sfd_ns=tx_sfd,tx_eof_ns=tx_eof,
                             rx_sfd_ns=rx_sfd,rx_eof_ns=rx_eof,app_first_ns=app_first,app_last_ns=app_last,**stages))
    sizes = (17,465,977,1471)
    for msg_id, (_,vl,length,_) in offered.items():
        r, v = divmod(msg_id,n_vl)
        require(vl == v+1 and length == sizes[(r+v)%4], "source traffic/length contract")
    require(set(lengths) == set(range(1,n_vl+1)) and
            all(s == {64,512,1024,1518} for s in lengths.values()), "per-VL four-length coverage")
    end_time, n_offer, n_commit, n_delivery, n_drop, n_bad, rm, mac_error, hdr, csum, ic, tb_errors = records["END"][0]
    require((n_offer,n_commit,n_delivery) == (count,count,count), "END sample counts")
    dropped = sum(p[2] == 1 for p in plans.values())
    bad = sum(p[2] == 2 for p in plans.values())
    require((n_drop,n_bad,rm,mac_error) == (dropped,bad,2*count-dropped-bad-count,bad), "physical/logical conservation")
    require(hdr == csum == ic == tb_errors == 0, "DUT/TB functional errors")
    require(end_time-max(t[1] for t in incoming.values()) >= 200_000, "insufficient drain window")
    require(end_time >= max(m["app_last_ns"] for m in messages), "completion precedes delivery")
    per_vl = {str(vl): dict(samples=len(values),max_ns=max(values),jitter_ns=max(values)-min(values))
              for vl,values in sorted(e2e_per_vl.items())}
    worst_vl = max(per_vl, key=lambda v:per_vl[v]["jitter_ns"])
    fixed = max((max(v)-min(v) for v in fixed_per_vl.values() if len(v)>1), default=None)
    finish = max(m["app_last_ns"] for m in messages)
    start = min(r[3] for r in offered.values())
    throughput = sum(m["payload_bytes"] for m in messages)*8*1000/(finish-start)
    jitter = per_vl[worst_vl]["jitter_ns"]
    summary = dict(scenario=scenario,topology="application TX -> independent A/B store-forward model -> application RX",
                   functional_audit="PASS",messages=count,tx_vl_exercised=n_vl,production_tx_vl=128,production_rx_vl=4096,
                   wire_body_lengths=[64,512,1024,1518],network_extra_delay_ns=extra,
                   dropped_copies=dropped,corrupted_copies=bad,rm_discarded=rm,
                   effective_payload_Mbps=throughput,throughput_window_ns=finish-start,
                   unique_payload_bytes=sum(m["payload_bytes"] for m in messages),
                   same_vl_jitter_max_ns=jitter,same_vl_jitter_worst_vl=int(worst_vl),
                   same_vl_same_length_jitter_max_ns=fixed,per_vl=per_vl,
                   stages={key:distribution([m[key] for m in messages]) for key in stages},
                   guards=dict(application_path_2ms_engineering=max(m["total_ns"] for m in messages)<=2_000_000,
                               same_vl_50us_engineering=jitter<=50_000,
                               payload_100Mbps=(throughput>=100) if scenario==1 else None),
                   official_GMII_latency="NOT_CLAIMED_BY_THIS_TOPOLOGY",official_jitter_numeric_limit="UNSPECIFIED")
    return summary, sorted(messages,key=lambda m:m["id"])


def checker_selftest(records):
    audit(records)
    mutations = []
    def mutate(tag, action):
        data = copy.deepcopy(records)
        action(data[tag])
        mutations.append(data)
    mutate("D", lambda rows:rows.pop())
    mutate("D", lambda rows:rows.append(rows[0]))
    mutate("W", lambda rows:rows.pop())
    mutate("I", lambda rows:rows.pop())
    mutate("N", lambda rows:rows.__setitem__(0,(*rows[0][:2],1,rows[0][3])))
    mutate("D", lambda rows:rows.__setitem__(0,(*rows[0][:-1],bytes([rows[0][-1][0]^1])+rows[0][-1][1:])))
    mutate("W", lambda rows:rows.__setitem__(0,(*rows[0][:-1],rows[0][-1][:-1]+bytes([rows[0][-1][-1]^1]))))
    mutate("I", lambda rows:rows.__setitem__(0,(rows[0][0],rows[0][1]-8,rows[0][2],rows[0][3])))
    mutate("F", lambda rows:rows.__setitem__(0,(rows[0][0],10**12)))
    mutate("END", lambda rows:rows.clear())
    for i,data in enumerate(mutations):
        try:
            audit(data)
        except ValueError:
            continue
        raise ValueError(f"negative checker selftest {i} incorrectly passed")
    late = copy.deepcopy(records)
    late["D"] = [(*r[:5],r[5]+3_000_000,r[6]+3_000_000,r[7]) for r in late["D"]]
    late["END"][0] = (late["END"][0][0]+3_000_000,*late["END"][0][1:])
    late_summary, _ = audit(late)
    require(late_summary["guards"]["application_path_2ms_engineering"] is False, "late delivery guard failed")
    print(f"Checker positive replay + {len(mutations)} negative mutations + 2ms guard test PASS")


def compare_delay(directory):
    baseline = read_capture(directory / "app_loopback_0.txt")
    delayed = read_capture(directory / "app_loopback_4.txt")
    _, before = audit(baseline)
    _, after = audit(delayed)
    require(all(baseline[tag] == delayed[tag] for tag in ("O","F","C","W")),
            "delay control changed the source/RTL TX trace")
    before_wire = {(r[0],int.from_bytes(r[3][50:54],"big")):r for r in baseline["I"]}
    after_wire = {(r[0],int.from_bytes(r[3][50:54],"big")):r for r in delayed["I"]}
    require(before_wire.keys() == after_wire.keys(), "delay control physical sample identity")
    for key, old in before_wire.items():
        new = after_wire[key]
        require(new[3] == old[3] and new[1]-old[1] == new[2]-old[2] == 5000,
                "configured +5us must appear at every physical RX boundary")
    changes = [b["total_ns"]-a["total_ns"] for a,b in zip(before,after)]
    summary = dict(status="PASS",physical_samples=len(before_wire),added_network_delay_ns=5000,
                   source_and_RTL_TX_unchanged=True,
                   application_total_change_ns=dict(min=min(changes),max=max(changes)),
                   note="Application changes may also include shared RX arbitration/scan phase effects.")
    (directory / "app_loopback_delay_control_summary.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8")
    print(f"Network +5us control PASS: {len(before_wire)} frames shifted exactly 5000ns; application delta={summary['application_total_change_ns']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario",type=int,choices=range(6))
    parser.add_argument("--directory",type=Path,default=Path("."))
    parser.add_argument("--selftest",action="store_true")
    parser.add_argument("--compare-delay",action="store_true")
    args = parser.parse_args()
    if args.compare_delay:
        compare_delay(args.directory)
        return 0
    require(args.scenario is not None, "--scenario or --compare-delay is required")
    prefix = args.directory / f"app_loopback_{args.scenario}"
    records = read_capture(prefix.with_suffix(".txt"))
    require(records["CONFIG"][0][0] == args.scenario, "wrong scenario capture")
    summary, messages = audit(records)
    summary["capture_sha256"] = hashlib.sha256(prefix.with_suffix(".txt").read_bytes()).hexdigest()
    if args.selftest:
        checker_selftest(records)
    Path(f"{prefix}_summary.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8")
    with Path(f"{prefix}_messages.csv").open("w",newline="",encoding="utf-8") as stream:
        writer = csv.DictWriter(stream,fieldnames=list(messages[0]))
        writer.writeheader()
        writer.writerows(messages)
    print(f"scenario={args.scenario} functional=PASS messages={summary['messages']} "
          f"total_max_us={summary['stages']['total_ns']['max_ns']/1000:.3f} "
          f"same_vl_jitter_us={summary['same_vl_jitter_max_ns']/1000:.3f} "
          f"unique_payload_Mbps={summary['effective_payload_Mbps']:.3f} guards={summary['guards']}")
    return 1 if any(v is False for v in summary["guards"].values()) else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError,ValueError,IndexError,KeyError) as error:
        print(f"CAPTURE AUDIT FAILED: {error}")
        raise SystemExit(2)
