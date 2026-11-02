"""Independent Ethernet/IP/UDP reference helpers for current endpoint captures."""
import binascii

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
