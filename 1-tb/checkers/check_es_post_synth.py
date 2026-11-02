"""Independent frame/application audit and RTL-versus-netlist capture comparison."""
from pathlib import Path
import argparse
import binascii
import copy
import hashlib
import json


def require(condition, message):
    if not condition:
        raise ValueError(message)


def folded(data):
    data += b'\0' * (len(data) % 2)
    value = sum(int.from_bytes(data[i:i+2], 'big') for i in range(0, len(data), 2))
    while value >> 16:
        value = (value & 65535) + (value >> 16)
    return value


def pattern(tag, length):
    return bytes((tag*17+p*7+(p>>8)) & 255 for p in range(length))


def read_capture(path):
    records = {k: [] for k in ('C', 'S', 'I', 'T', 'R', 'E', 'RESULT')}
    for line in path.read_text(encoding='utf-8').splitlines():
        kind, *values = line.split()
        require(kind in records, 'unknown capture record')
        records[kind].append(values)
    return records


def messages(records):
    result, data = [], bytearray()
    for cycle, value, last in records:
        data.append(int(value, 16))
        if int(last):
            require(len(data) >= 3, 'message header/payload length')
            result.append((int.from_bytes(data[:2], 'big'), bytes(data[2:])))
            data.clear()
    require(not data, 'partial message at end')
    return result


def frames(records, incoming=False):
    lanes, output = {}, []
    for row in records:
        cycle, lane, value = int(row[0]), int(row[1]), int(row[2], 16)
        if lane in lanes and lanes[lane][0]+1 != cycle:
            _, start, raw, er = lanes.pop(lane)
            output.append((lane, start, bytes(raw), er))
        if lane not in lanes:
            lanes[lane] = [cycle-1, cycle, bytearray(), False]
        lanes[lane][0] = cycle
        lanes[lane][2].append(value)
        lanes[lane][3] |= bool(int(row[3])) if incoming else False
    output += [(lane, start, bytes(raw), er) for lane, (_, start, raw, er) in lanes.items()]
    return sorted(output, key=lambda f: (f[1], f[0]))


def decode(raw):
    require(raw[:8] == b'\x55'*7+b'\xd5', 'preamble/SFD')
    mac = raw[8:]
    require(64 <= len(mac) <= 1518, 'Ethernet frame length')
    require(binascii.crc32(mac[:-4]) == int.from_bytes(mac[-4:], 'little'), 'FCS')
    require(mac[12:14] == b'\x08\x00' and mac[14] == 0x45, 'EtherType/IPv4')
    ip = mac[14:]
    require(folded(ip[:20]) == 65535 and ip[9] == 17, 'IPv4 checksum/protocol')
    size = int.from_bytes(ip[2:4], 'big')
    require(size >= 20 and 14+size+1+4 <= len(mac), 'IP length/SN boundary')
    return int.from_bytes(mac[4:6], 'big'), ip[:20], ip[20:size], mac[-5]


def audit(records):
    require(len(records['RESULT']) == 1, 'missing completion record')
    checks, errors, na, nb, deliveries, drops = map(int, records['RESULT'][0])
    require(checks > 10000 and errors == 0 and deliveries == 7 and drops == 2, 'TB result')
    submitted = messages(records['S'])
    require(len(submitted) == len(records['C']) == 8, 'all input cases recorded')
    for (vl, payload), (_, offered_vl, n, tag, mode) in zip(submitted, records['C']):
        require(vl == int(offered_vl, 16) and payload == pattern(int(tag), int(n)), 'source payload')
    expected = {0x1001: (1, 1), 0x1002: (17, 2), 0x1003: (63, 3),
                0x1004: (1471, 4), 0x1005: (31, 5), 0x100a: (40, 10), 0x100c: (40, 12)}
    delivered = [m for m in messages(records['R']) if m[0] != 0x1000]
    require(len(delivered) == 7 and len({vl for vl, _ in delivered}) == 7, 'exactly once delivery')
    require(set(vl for vl, _ in delivered) == set(expected), 'delivery VL set')
    for vl, payload in delivered:
        n, tag = expected[vl]
        require(payload == pattern(tag, n), 'delivered payload')
    outgoing = frames(records['T'])
    require(sum(f[0] == 0 for f in outgoing) == na and sum(f[0] == 1 for f in outgoing) == nb, 'wire frame counts')
    by_vl = {}
    for lane, cycle, raw, er in outgoing:
        vl, ip, udp, sn = decode(raw)
        require(not er, 'unexpected transmit error')
        if vl == 0x1000:
            continue
        require(0x1001 <= vl <= 0x1006, 'transmitted VL')
        n, tag = [(1,1),(17,2),(63,3),(1471,4),(31,5),(33,6)][vl-0x1001]
        require(len(udp) == n+8 and udp[8:] == pattern(tag,n), 'wire UDP payload')
        require(int.from_bytes(udp[4:6], 'big') == n+8, 'UDP length')
        require(int.from_bytes(udp[:2], 'big') == 1000+vl-0x1000 and
                int.from_bytes(udp[2:4], 'big') == 2000+vl-0x1000, 'UDP endpoints')
        require(folded(ip[12:20]+b'\0\x11'+udp[4:6]+udp) == 65535, 'UDP checksum')
        require(sn == 1, 'initial per-VL sequence number')
        by_vl.setdefault(vl, []).append(lane)
    require(by_vl == {vl: [0,1] for vl in range(0x1001,0x1007)}, 'two copies of every submitted valid message')
    bad_crc = 0
    errors_on_wire = 0
    fragments = {}
    for lane, cycle, raw, er in frames(records['I'], incoming=True):
        errors_on_wire += er
        try:
            vl, ip, data, sn = decode(raw)
        except ValueError as error:
            require(str(error) in ('FCS',), 'unexpected malformed injected frame: '+str(error))
            bad_crc += 1
            continue
        if vl in (0x100a, 0x100b, 0x100c):
            flags = int.from_bytes(ip[6:8], 'big')
            offset = (flags & 8191)*8
            fragments.setdefault((lane,vl), []).append((offset, data, ip))
    require(bad_crc == 3 and errors_on_wire == 1, 'CRC and RX_ER faults really exercised')
    require(len(fragments) == 6, 'three fragmented datagrams on two networks')
    for (lane, vl), pieces in fragments.items():
        require(len(pieces) == 2 and sorted(p[0] for p in pieces) == [0,32], 'fragment coverage')
        if vl == 0x100c:
            require([p[0] for p in pieces] == [32,0], 'out-of-order fragment injection')
        udp = b''.join(p[1] for p in sorted(pieces, key=lambda p:p[0]))
        ip = pieces[0][2]
        good = folded(ip[12:20]+b'\0\x11'+udp[4:6]+udp) == 65535
        require(good == (vl != 0x100b), 'whole-datagram checksum good/bad control')
    return dict(checks=checks, transmitted_frames=len(outgoing), logical_deliveries=7,
                application_rejections=drops, bad_crc_inputs=bad_crc, rx_er_inputs=errors_on_wire,
                fragment_datagrams=len(fragments), result='PASS')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    parser.add_argument('--selftest', action='store_true')
    args = parser.parse_args()
    captures = [args.directory/(mode+'_capture.txt') for mode in ('rtl','netlist')]
    records = [read_capture(p) for p in captures]
    results = [audit(r) for r in records]
    require(records[0] == records[1], 'RTL/netlist external event stream differs (including cycles)')
    if args.selftest:
        for kind in ('wire_byte','delivery_missing','source_byte'):
            broken = copy.deepcopy(records[1])
            if kind == 'wire_byte':
                broken['T'][20][2] = f'{int(broken["T"][20][2],16)^1:02x}'
            elif kind == 'delivery_missing':
                broken['R'].pop()
            else:
                broken['S'][2][1] = f'{int(broken["S"][2][1],16)^1:02x}'
            try:
                audit(broken)
            except ValueError:
                continue
            raise ValueError('negative control not rejected: '+kind)
    result = dict(result='PASS', rtl=results[0], netlist=results[1],
                  cycle_exact_external_events=True, negative_controls='3/3 rejected' if args.selftest else 'not run',
                  capture_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in captures},
                  scope='Full production netlist, external functional test only; no STA, capacity or throughput claim')
    (args.directory/'comparison.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
