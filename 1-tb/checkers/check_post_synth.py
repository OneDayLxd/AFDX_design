"""独立校验端口抓帧，并逐帧对比生产 RTL 与综合后网表输出。"""
import argparse
import binascii
from pathlib import Path


def folded_sum(data):
    if len(data) & 1:
        data += b"\0"
    total = sum(int.from_bytes(data[i:i + 2], "big") for i in range(0, len(data), 2))
    while total >> 16:
        total = (total & 0xffff) + (total >> 16)
    return total


def load_capture(path):
    records = [[], []]
    for line_no, line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), 1):
        fields = line.split()
        if not fields:
            continue
        assert fields[0] == "F", (path, line_no, "unknown record")
        timestamp, net, count = map(int, fields[1:4])
        data = bytes.fromhex(" ".join(fields[4:]))
        assert net in (0, 1) and len(data) == count, (path, line_no, "capture length")
        assert data[:8] == b"\x55" * 7 + b"\xd5", (path, line_no, "preamble/SFD")
        mac = data[8:]
        assert 64 <= len(mac) <= 1518, (path, line_no, "MAC length")
        assert binascii.crc32(mac[:-4]).to_bytes(4, "little") == mac[-4:], (path, line_no, "FCS")
        assert mac[12:14] == b"\x08\x00" and mac[14] == 0x45, (path, line_no, "IPv4")
        assert folded_sum(mac[14:34]) == 0xffff, (path, line_no, "IP checksum")
        ip_len = int.from_bytes(mac[16:18], "big")
        udp_len = int.from_bytes(mac[38:40], "big")
        assert ip_len == 20 + udp_len and 14 + ip_len <= len(mac) - 5, (path, line_no, "IP/UDP length")
        pseudo = mac[26:34] + bytes([0, 17]) + udp_len.to_bytes(2, "big")
        assert mac[40:42] != b"\0\0", (path, line_no, "UDP checksum enabled")
        assert folded_sum(pseudo + mac[34:34 + udp_len]) == 0xffff, (path, line_no, "UDP checksum")
        assert mac[-5] != 0, (path, line_no, "AFDX sequence number")
        records[net].append((timestamp, data))
    assert all(records), (path, "both networks must produce frames")
    return records


def compare(reference, actual):
    for net in (0, 1):
        assert len(reference[net]) == len(actual[net]), (net, "frame count mismatch")
        for index, ((_, expected), (_, observed)) in enumerate(zip(reference[net], actual[net])):
            assert expected == observed, (net, index, "RTL/netlist frame bytes mismatch")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference")
    parser.add_argument("netlist")
    parser.add_argument("--smoke", action="store_true", help="check three four-VL smoke phases")
    args = parser.parse_args()
    reference = load_capture(args.reference)
    actual = load_capture(args.netlist)
    compare(reference, actual)
    if args.smoke:
        for net in (0, 1):
            assert len(actual[net]) == 12, (net, "three phases must each transmit four frames")
            for start in (0, 4, 8):
                vlids = {int.from_bytes(data[12:14], "big")
                         for _, data in actual[net][start:start + 4]}
                assert vlids == {0x1000, 0x1001, 0x1002, 0x1003}, (net, start, "VL coverage")
    print(f"PASS: independent FCS/IP/UDP/SN checks and byte-exact RTL/netlist comparison; "
          f"A/B frames={len(actual[0])}/{len(actual[1])}")


if __name__ == "__main__":
    main()
