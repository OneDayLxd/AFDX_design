"""GMII input/output same-frame audit. Missing output is never a latency PASS."""
import argparse
import binascii
import json
from collections import defaultdict
from pathlib import Path


def require(condition, message):
    if not condition:
        raise ValueError(message)


def frame_key(frame):
    require(64 <= len(frame) <= 1518, "Ethernet frame length")
    require(frame[:4] == b"\x03\x00\x00\x00" and frame[12:14] == b"\x08\x00", "AFDX destination/type")
    require(int.from_bytes(frame[-4:], "little") == binascii.crc32(frame[:-4]), "FCS mismatch")
    require(frame[14] == 0x45 and frame[23] == 17, "IPv4/UDP header")
    total = sum(int.from_bytes(frame[i:i+2], "big") for i in range(14, 34, 2))
    while total >> 16:
        total = (total & 65535) + (total >> 16)
    require(total == 65535, "IPv4 checksum")
    vl = int.from_bytes(frame[4:6], "big")
    return vl, frame[-5]


def audit(records, end, scenario):
    incoming, outgoing = {}, {}
    lengths = defaultdict(set)
    previous = {}
    for direction, net, sfd, eof, frame in records:
        require(direction in ("IN", "OUT") and net in (0, 1), "capture direction/network")
        vl, sn = frame_key(frame)
        require(eof - sfd == (len(frame)+1)*8, "SFD/EOF byte timing")
        key = net, vl, sn
        target = incoming if direction == "IN" else outgoing
        require(key not in target, "duplicate frame identity")
        target[key] = (sfd, eof, frame)
        if direction == "IN":
            if (net, vl) in previous:
                require(sfd - previous[net, vl] >= 2_000_000, "source VL violates BAG")
            previous[net, vl] = sfd
            lengths[vl].add(len(frame))
    n_vl = 1 if scenario == 0 else 16
    require(set(lengths) == set(range(0x1001, 0x1001+n_vl)), "input VL coverage")
    require(len(incoming) == n_vl*8*2, "input frame count")
    require(all(s == {64,512,1024,1518} for s in lengths.values()), "four required lengths per VL")
    logical = {(vl, sn) for net, vl, sn in incoming}
    for vl, sn in logical:
        a, b = incoming[0, vl, sn], incoming[1, vl, sn]
        require(abs(a[0]-b[0]) <= 500_000, "A/B source skew")
        # A/B 源 MAC 接口号及其 FCS 不同，其他内容必须相同。
        require(a[2][:11]+a[2][12:-4] == b[2][:11]+b[2][12:-4], "A/B logical copy mismatch")
    require(end is not None and end[0] - max(r[1] for r in incoming.values()) > 2_000_000,
            "capture ended before last deadline")
    require(end[1] == len(logical) and end[2] == len(logical), "RX delivery/RM conservation")
    require(all(value == 0 for value in end[3:]), "RX error/drop counter")
    delays = defaultdict(list)
    matched, unrelated = set(), []
    for key, (sfd, eof, frame) in outgoing.items():
        if key not in incoming:
            unrelated.append(key)
            continue
        original = incoming[key]
        require(frame == original[2], "same identity but frame bytes changed")
        require(sfd >= original[0], "output precedes input")
        delays[key[0], key[1]].append(sfd-original[0])
        matched.add(key)
    missing = sorted(set(incoming)-matched)
    all_delays = [d for values in delays.values() for d in values]
    # 只有完整匹配全部输入，才给出正式的最大值和抖动结论。
    complete = not missing
    maximum = max(all_delays) if complete else None
    jitter = max(max(v)-min(v) for v in delays.values()) if complete else None
    status = "NO_MATCHED_OUTPUT" if not matched else "MISSING_OUTPUT" if missing else "MEASURED"
    return dict(scenario=scenario, status=status, input_logical=len(logical),
                input_physical=len(incoming), output_physical=len(outgoing),
                matched_physical=len(matched), unrelated_outputs=len(unrelated), unrelated_keys=unrelated,
                missing_physical=len(missing), missing_keys=missing,
                rx_delivered=end[1], rm_discarded=end[2],
                max_latency_ns=maximum, max_same_vl_jitter_ns=jitter,
                latency_2ms="NOT_MEASURED" if not complete else "PASS" if maximum<=2_000_000 else "FAIL",
                jitter_50us_engineering="NOT_MEASURED" if not complete else "PASS" if jitter<=50_000 else "FAIL",
                official_jitter_numeric_limit="UNSPECIFIED",
                same_frame_identity="network + VLID + SN, then exact Ethernet bytes including FCS")


def selftest():
    records = []
    for r in range(8):
        length = [64,512,1024,1518][r%4]
        for net in range(2):
            frame = bytearray(length-4)
            frame[:6] = bytes.fromhex("030000001001")
            frame[11] = 0x21+net
            frame[12:15] = bytes.fromhex("080045")
            frame[23] = 17
            frame[24:26] = b"\x00\x00"
            total = sum(int.from_bytes(frame[i:i+2],"big") for i in range(14,34,2))
            while total>>16:
                total = (total&65535)+(total>>16)
            frame[24:26] = (65535-total).to_bytes(2,"big")
            frame[-1] = r+1
            frame = bytes(frame)+binascii.crc32(frame).to_bytes(4,"little")
            sfd = r*2_500_000 + net*64
            records.append(("IN",net,sfd,sfd+(length+1)*8,frame))
            records.append(("OUT",net,sfd+20_000,sfd+20_000+(length+1)*8,frame))
    end = (25_000_000,8,8,0,0,0,0)
    good = audit(records,end,0)
    require(good["max_latency_ns"]==20_000 and good["max_same_vl_jitter_ns"]==0,"positive timing oracle")
    missing = audit([r for r in records if r[0]=="IN"],end,0)
    require(missing["max_latency_ns"] is None and missing["latency_2ms"]=="NOT_MEASURED","missing output must not pass")
    partial = audit(records[:-1],end,0)
    require(partial["status"]=="MISSING_OUTPUT" and partial["max_latency_ns"] is None,"partial sample exclusion")
    extra = bytearray(records[1][4][:-4]); extra[5]=2
    extra = bytes(extra)+binascii.crc32(extra).to_bytes(4,"little")
    unrelated = ("OUT",0,20_000,20_000+(len(extra)+1)*8,extra)
    noise = audit([r for r in records if r[0]=="IN"]+[unrelated],end,0)
    require(noise["matched_physical"]==0 and noise["unrelated_outputs"]==1 and noise["latency_2ms"]=="NOT_MEASURED",
            "unrelated output must not satisfy timing")
    for corrupt in ("CRC","IDENTITY","DUPLICATE","SHORT_CAPTURE","DEADLINE","JITTER"):
        rows = list(records)
        final = end
        if corrupt == "CRC":
            r = rows[-1]; rows[-1] = (*r[:4], r[4][:-1]+bytes([r[4][-1]^1]))
        elif corrupt == "IDENTITY":
            r = rows[-1]; f = bytearray(r[4][:-4]); f[6]^=1
            rows[-1] = (*r[:4], bytes(f)+binascii.crc32(f).to_bytes(4,"little"))
        elif corrupt == "DUPLICATE":
            rows.append(rows[-1])
        elif corrupt == "SHORT_CAPTURE":
            final = (1,*end[1:])
        else:
            r = rows[-1]; delta=2_000_001 if corrupt=="DEADLINE" else 50_001
            rows[-1] = (r[0],r[1],r[2]+delta,r[3]+delta,r[4])
        try:
            result = audit(rows,final,0)
        except ValueError:
            require(corrupt not in ("DEADLINE","JITTER"),"unexpected timing parser rejection")
        else:
            field = "latency_2ms" if corrupt=="DEADLINE" else "jitter_50us_engineering"
            require(corrupt in ("DEADLINE","JITTER") and result[field]=="FAIL","bad capture accepted")
    print("GMII boundary checker selftest: positive, missing, partial, unrelated output, identity, CRC, duplicate, timeout, deadline, jitter PASS")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario",type=int,choices=(0,1),default=0)
    parser.add_argument("--selftest",action="store_true")
    args = parser.parse_args()
    if args.selftest:
        selftest()
    else:
        path = Path(f"gmii_boundary_{args.scenario}")
        rows, end = [], None
        for line in path.with_suffix(".txt").read_text().splitlines():
            parts = line.split()
            if parts[0]=="END":
                require(end is None,"duplicate END")
                end = tuple(map(int,parts[1:]))
            else:
                require(end is None,"capture records after END")
                rows.append((parts[0],int(parts[1]),int(parts[2]),int(parts[3]),bytes.fromhex(parts[4])))
        result = audit(rows,end,args.scenario)
        Path(f"{path}_summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
        print(json.dumps({k:v for k,v in result.items() if k!="missing_keys"},indent=2))
        raise SystemExit(0 if result["latency_2ms"]=="PASS" and result["jitter_50us_engineering"]=="PASS" else 1)
