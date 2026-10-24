"""只读 GMII 管理帧检查器，不运行仿真、不修改 RTL 或激励。
使用 CCAnalysis Python，参数：场景名 捕获文件。
"""
import argparse
import binascii
import struct
import sys
import zlib
from pathlib import Path
import mib_model as mibgen

N_TX_VL = 4                      # tb/afdx_mgmt_test_config.v
PRIV = (1, 3, 6, 1, 3, 664)
MGR_PORT = 50161                 # 管理站源端口
RTC_PORT = 3190
TID_BASE = 0xC000

# ---------------------------------------------------------------- BER
INTEGER, OCTETS, NULL, OID, SEQ = 0x02, 0x04, 0x05, 0x06, 0x30
COUNTER32, GAUGE32, TIMETICKS, COUNTER64 = 0x41, 0x42, 0x43, 0x46
NO_SUCH_OBJECT, NO_SUCH_INSTANCE, END_OF_MIB = 0x80, 0x81, 0x82
GET, GETNEXT, RESPONSE, SET, TRAP2 = 0xA0, 0xA1, 0xA2, 0xA3, 0xA7


def enc_len(n):
    if n < 0x80:
        return bytes([n])
    if n < 0x100:
        return bytes([0x81, n])
    return bytes([0x82, n >> 8, n & 0xFF])


def tlv(tag, content):
    return bytes([tag]) + enc_len(len(content)) + content


def enc_int(v):
    n = 1
    while not -(1 << (8 * n - 1)) <= v < (1 << (8 * n - 1)):
        n += 1
    return (v & ((1 << (8 * n)) - 1)).to_bytes(n, 'big')


def enc_uint(v):
    b = v.to_bytes(max(1, (v.bit_length() + 7) // 8), 'big')
    return b'\x00' + b if b[0] & 0x80 else b


def enc_oid(arcs):
    out = bytearray([40 * arcs[0] + arcs[1]])
    for a in arcs[2:]:
        chunk = [a & 0x7F]
        a >>= 7
        while a:
            chunk.append(0x80 | (a & 0x7F))
            a >>= 7
        out += bytes(reversed(chunk))
    return bytes(out)


def dec_tlv(buf, pos, end=None):
    end = len(buf) if end is None else end
    if pos + 2 > end:
        raise ValueError('truncated TLV')
    tag, first = buf[pos], buf[pos + 1]
    pos += 2
    if first < 0x80:
        n = first
    else:
        k = first & 0x7F
        if k == 0 or k > 2 or pos + k > end:
            raise ValueError('bad length')
        n = int.from_bytes(buf[pos:pos + k], 'big')
        if (k == 1 and n < 0x80) or (k == 2 and n < 0x100):
            raise ValueError('non-minimal length')
        pos += k
    if pos + n > end:
        raise ValueError('TLV exceeds parent')
    return tag, buf[pos:pos + n], pos + n


def dec_oid(c):
    arcs = [min(c[0] // 40, 2), c[0] - 40 * min(c[0] // 40, 2)]
    v = 0
    for b in c[1:]:
        v = (v << 7) | (b & 0x7F)
        if not b & 0x80:
            arcs.append(v)
            v = 0
    return tuple(arcs)


def dec_int(c, signed=True, strict=True):
    if strict and len(c) > 1 and ((c[0] == 0 and not c[1] & 0x80) or (c[0] == 0xFF and c[1] & 0x80)):
        raise ValueError('non-minimal INTEGER')
    return int.from_bytes(c, 'big', signed=signed)


def snmp_request(pdu, reqid, varbinds, community=b'public', es=0, ei=0, version=1):
    vbl = b''.join(tlv(SEQ, tlv(OID, enc_oid(o)) + v) for o, v in varbinds)
    body = tlv(INTEGER, enc_int(reqid)) + tlv(INTEGER, enc_int(es)) + tlv(INTEGER, enc_int(ei)) + tlv(SEQ, vbl)
    return tlv(SEQ, tlv(INTEGER, enc_int(version)) + tlv(OCTETS, community) + tlv(pdu, body))


def parse_snmp(msg, strict=True):
    """严格解析 SNMPv2c 消息，返回 dict；任何编码错误都抛出异常。
    strict=False 只放宽 VarBind 值的最短编码检查（失败 SET 的响应按原样回显请求值）。"""
    tag, c, end = dec_tlv(msg, 0)
    if tag != SEQ or end != len(msg):
        raise ValueError('bad message')
    t, ver, p = dec_tlv(c, 0)
    t2, comm, p = dec_tlv(c, p)
    ptag, pdu, p = dec_tlv(c, p)
    if p != len(c) or t != INTEGER or t2 != OCTETS:
        raise ValueError('bad header')
    _, rid, q = dec_tlv(pdu, 0)
    _, es, q = dec_tlv(pdu, q)
    _, ei, q = dec_tlv(pdu, q)
    vt, vbl, q = dec_tlv(pdu, q)
    if q != len(pdu) or vt != SEQ:
        raise ValueError('bad PDU')
    vbs, r = [], 0
    while r < len(vbl):
        st, vb, r = dec_tlv(vbl, r)
        ot, oc, s = dec_tlv(vb, 0)
        tt, tc, s = dec_tlv(vb, s)
        if st != SEQ or ot != OID or s != len(vb):
            raise ValueError('bad varbind')
        if tt in (INTEGER,):
            val = dec_int(tc, strict=strict)
        elif tt in (COUNTER32, GAUGE32, TIMETICKS, COUNTER64):
            if strict and len(tc) > 1 and tc[0] == 0 and not tc[1] & 0x80:
                raise ValueError('non-minimal unsigned')
            val = int.from_bytes(tc, 'big')
        elif tt == OID:
            val = dec_oid(tc)
        else:
            val = bytes(tc)
        vbs.append((dec_oid(oc), tt, val))
    return {'version': dec_int(ver), 'community': bytes(comm), 'pdu': ptag, 'rid': dec_int(rid),
            'es': dec_int(es), 'ei': dec_int(ei), 'vbs': vbs}


# ---------------------------------------------------------------- 帧
def csum(data):
    if len(data) % 2:
        data += b'\x00'
    s = sum(struct.unpack('!%dH' % (len(data) // 2), data))
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return s


def parse_frame(fr):
    """校验 AFDX 帧：FCS、目的 MAC、以太网类型、IPv4 首部/校验和、UDP 长度/校验和。"""
    if binascii.crc32(fr[:-4]) != int.from_bytes(fr[-4:], 'little'):
        raise ValueError('FCS')
    if fr[0:4] != b'\x03\x00\x00\x00' or fr[12:14] != b'\x08\x00' or fr[14] != 0x45:
        raise ValueError('header')
    vlid = int.from_bytes(fr[4:6], 'big')
    ip_len = int.from_bytes(fr[16:18], 'big')
    if csum(fr[14:34]) != 0xFFFF or fr[23] != 17 or fr[30:32] != b'\xe0\xe0' or fr[32:34] != fr[4:6]:
        raise ValueError('ip')
    sport, dport, ulen, uck = struct.unpack('!HHHH', fr[34:42])
    if ulen != ip_len - 20:
        raise ValueError('udp length')
    payload = fr[42:42 + ulen - 8]
    pseudo = fr[26:34] + b'\x00\x11' + ulen.to_bytes(2, 'big')
    if uck and csum(pseudo + fr[34:42 + ulen - 8]) != 0xFFFF:
        raise ValueError('udp checksum')
    return {'vlid': vlid, 'sport': sport, 'dport': dport, 'payload': bytes(payload),
            'sn': fr[max(42 + ulen - 8, 59)], 'len': len(fr)}


def read_capture(path):
    frames, marks, samples = [], [], []
    for line in path.read_text().splitlines():
        f = line.split()
        if f[0] == 'F':
            raw = bytes(int(x, 16) for x in f[4:])
            rec = parse_frame(raw)
            rec.update(start=int(f[1]), end=int(f[2]))
            frames.append(rec)
        elif f[0] == 'P':
            marks.append((int(f[1]), len(frames)))
        elif f[0] == 'Q':
            samples.append({'t': int(f[1]), 'rtc': int(f[2]), 'master': int(f[3]), 'state': int(f[4])})
    return frames, marks, samples


# ---------------------------------------------------------------- 脚本
class Script:
    def __init__(self):
        self.lines = []
        self.expect = 0

    def send(self, payload, dport, sport=MGR_PORT):
        self.lines.append(f'S {sport} {dport} {len(payload)} ' + ' '.join(f'{b:02x}' for b in payload))

    def wait(self, us):
        self.lines.append(f'W {us}')

    def recv(self, n=1, timeout_us=20000):
        self.expect += n
        self.lines.append(f'R {n} {timeout_us}')

    def mark(self, tag):
        self.lines.append(f'P {tag}')

    def raw(self, line):
        self.lines.append(line)

    # 时间主站（测试平台内置模型）
    def master(self, drift_ppb, offset_us):
        self.lines.append(f'T {drift_ppb} {offset_us}')

    def auto_resp(self, on):
        self.lines.append(f'A {int(on)}')

    def sync(self, seq, src=0x0A01, q=10, logp=13, status=0, reply=True):
        self.lines.append(f'Y {seq & 0xFFFFFFFF} {src} {q} {logp} {status}')
        if reply:
            self.recv(1)

    def oneway(self, seq, src=0x0A01, q=10, logp=13):
        self.lines.append(f'O {seq & 0xFFFFFFFF} {src} {q} {logp}')

    def dup(self):
        self.lines.append('D')

    def sample(self):
        self.lines.append('Q')

    def text(self):
        return '\n'.join(self.lines + ['E']) + '\n'


CAPTURE = None
def run(script, name):
    """只读当前场景的捕获文件，复用场景的独立协议断言。"""
    return read_capture(CAPTURE)

class Checker:
    def __init__(self, name):
        self.name, self.checks, self.errors = name, 0, []

    def ok(self, cond, msg):
        self.checks += 1
        if not cond:
            self.errors.append(msg)
            print(f'  FAIL {self.name}: {msg}')
        return cond

    def report(self):
        status = 'PASS' if not self.errors else 'FAIL'
        print(f'{self.name}: {self.checks} checks, {len(self.errors)} errors -> {status}')
        return {'checks': self.checks, 'errors': self.errors}


def entries():
    return sorted(mibgen.MIB, key=mibgen.full_oid)


def visible_oids():
    """GETNEXT 可见实例的字典序列表（表列展开为 VL 实例）。"""
    out = []
    for e in entries():
        name, base, suf, typ, acc = e[:5]
        if acc == 'hidden':
            continue
        oid = mibgen.full_oid(e)
        if base == 1 and len(suf) == 4:
            out += [(oid + (i,), name, typ) for i in range(1, N_TX_VL + 1)]
        else:
            out.append((oid, name, typ))
    return out


def oid_of(name, inst=None):
    e = next(e for e in entries() if e[0] == name)
    return mibgen.full_oid(e) + ((inst,) if inst is not None else ())


def snmp_payloads(frames, port=MGR_PORT, strict=True):
    return [parse_snmp(f['payload'], strict) for f in frames if f['dport'] == port and f['sport'] in (161, 1161)]


def out_before(frames, rid):
    """在请求 rid 的响应之前已发出的 SNMP 报文数（响应 + Trap），即该时刻的 snmpOutPkts。"""
    for i, f in enumerate(frames):
        if f['dport'] == MGR_PORT and parse_snmp(f['payload'], False)['rid'] == rid:
            return sum(1 for g in frames[:i] if g['sport'] in (161, 1161))
    return None


# ---------------------------------------------------------------- 场景：冒烟
def scenario_smoke():
    s = Script()
    s.send(snmp_request(GET, 0x1234, [(oid_of('sysDescr'), b'\x05\x00')]), 161)
    s.recv(1)
    frames, _, _ = run(s, 'smoke')
    c = Checker('smoke')
    rsp = snmp_payloads(frames)
    c.ok(len(rsp) == 1, 'one SNMP response')
    if rsp:
        r = rsp[0]
        c.ok(r['pdu'] == RESPONSE and r['rid'] == 0x1234 and r['es'] == 0, 'GetResponse echoes request-id')
        c.ok(r['vbs'][0][0] == oid_of('sysDescr') and r['vbs'][0][2] == b'AFDX End System FPGA', 'sysDescr value')
        rf = [f for f in frames if f['dport'] == MGR_PORT][0]
        c.ok(rf['vlid'] == 0x1000 and rf['sport'] == 161, 'reply from UDP 161 to the request source port on mgmt VL')
    return c.report()


# ---------------------------------------------------------------- 场景：SNMP
WCOMM = b'private'
VL_ROM = {'vlId': [0x1000, 0x1001, 0x1002, 0x1003], 'vlBagUs': [1000, 2000, 4000, 8000],
          'vlLmax': [1471, 200, 64, 128], 'vlNetEn': [3, 3, 1, 2], 'vlRomLmax': [1471, 200, 64, 128]}


def g32(v):
    return tlv(GAUGE32, enc_uint(v))


def integer(v):
    return tlv(INTEGER, enc_int(v))


class Snmp:
    """按顺序生成请求并记录期望的响应数；responses() 返回按请求顺序配对的解析结果。"""

    def __init__(self, script):
        self.s, self.rid, self.expect = script, 100, []

    def req(self, pdu, vbs, comm=b'public', port=161, reply=True):
        self.rid += 1
        self.s.send(snmp_request(pdu, self.rid, vbs, comm), port)
        if reply:
            self.s.recv(1)
            self.expect.append(self.rid)
        else:
            self.s.wait(400)
        return self.rid

    def get(self, *names_inst, **kw):
        return self.req(GET, [(oid_of(*n) if isinstance(n, tuple) else oid_of(n), b'\x05\x00')
                              for n in names_inst], **kw)

    def set(self, vbs, comm=WCOMM, **kw):
        return self.req(SET, [(oid_of(*n) if isinstance(n, tuple) else oid_of(n), v) for n, v in vbs],
                        comm, **kw)


def responses(frames, port=MGR_PORT, strict=True):
    return {r['rid']: r for r in snmp_payloads(frames, port, strict)}


def vals(r):
    return [v for _, _, v in r['vbs']]


def scenario_snmp_get():
    s = Script()
    q = Snmp(s)
    r_multi = q.get('sysDescr', 'sysObjectID', 'esIdentity', ('vlId', 1), ('vlBagUs', 3), ('vlLmax', 2),
                    ('vlNetEn', 4), 'snmpPort', 'trapDstPort', 'rtcPort')
    r_exc = q.req(GET, [(PRIV + (9, 9, 0), b'\x05\x00'), (oid_of('vlBagUs', 9), b'\x05\x00'),
                        (oid_of('vlBagUs', 0), b'\x05\x00'), (oid_of('sysDescr')[:-1], b'\x05\x00'),
                        (oid_of('sysDescr') + (0,), b'\x05\x00'), (oid_of('esFwVersion'), b'\x05\x00')])
    r_16 = q.req(GET, [(oid_of('vlId', 1 + i % 4), b'\x05\x00') for i in range(16)])
    r_17 = q.req(GET, [(oid_of('vlId', 1 + i % 4), b'\x05\x00') for i in range(17)])
    q.rid = -7                      # 负 request-id 与 4 字节 request-id 原样回显
    r_neg = q.req(GET, [(oid_of('sysName'), b'\x05\x00')])
    q.rid = 0x7FFFFFF0
    r_big = q.req(GET, [(oid_of('sysUpTime'), b'\x05\x00')])
    q.get('sysDescr', comm=b'publi', reply=False)
    q.get('sysDescr', comm=b'publicx', reply=False)
    r_cnt = q.get('snmpInPkts', 'snmpInBadCommunity', 'snmpInParseErrs', 'snmpOutPkts', 'mgmtRxAccepted')
    frames, _, _ = run(s, 'snmp_get')
    c = Checker('snmp_get')
    rs = responses(frames)
    c.ok(len(rs) == len(q.expect), f'{len(q.expect)} responses, got {len(rs)}')
    c.ok(all(f['vlid'] == 0x1000 and f['sport'] == 161 for f in frames if f['dport'] == MGR_PORT),
         'responses on management VL from UDP 161')
    if r_multi in rs:
        r = rs[r_multi]
        c.ok(r['es'] == 0 and r['ei'] == 0 and len(r['vbs']) == 10, 'multi-varbind GET ok')
        c.ok([o for o, _, _ in r['vbs']] == [oid_of('sysDescr'), oid_of('sysObjectID'), oid_of('esIdentity'),
                                              oid_of('vlId', 1), oid_of('vlBagUs', 3), oid_of('vlLmax', 2),
                                              oid_of('vlNetEn', 4), oid_of('snmpPort'), oid_of('trapDstPort'),
                                              oid_of('rtcPort')], 'varbind order preserved')
        c.ok(vals(r) == [b'AFDX End System FPGA', PRIV, 0x0101, 0x1000, 4000, 200, 2, 161, 162, RTC_PORT],
             f'multi-varbind values {vals(r)}')
        c.ok([t for _, t, _ in r['vbs']] == [OCTETS, OID, GAUGE32, GAUGE32, GAUGE32, GAUGE32, INTEGER,
                                             INTEGER, INTEGER, INTEGER], 'value types follow the MIB')
    if r_exc in rs:
        r = rs[r_exc]
        tags = [t for _, t, _ in r['vbs']]
        c.ok(r['es'] == 0 and tags[:2] == [NO_SUCH_OBJECT, NO_SUCH_INSTANCE], f'unknown object/instance {tags}')
        c.ok(tags[2] == NO_SUCH_INSTANCE and tags[3] in (NO_SUCH_OBJECT, NO_SUCH_INSTANCE)
             and tags[4] in (NO_SUCH_OBJECT, NO_SUCH_INSTANCE), f'instance 0 / missing .0 / extra arc {tags}')
        c.ok(tags[5] == GAUGE32 and all(v == b'' for _, t, v in r['vbs'][:5]), 'exceptions carry NULL content')
    if r_16 in rs:
        r = rs[r_16]
        c.ok(r['es'] == 0 and vals(r) == [0x1000 + i % 4 for i in range(16)], '16 varbinds (limit) served')
    if r_17 in rs:
        r = rs[r_17]
        c.ok(r['es'] == 1 and r['ei'] == 0 and r['vbs'] == [], '17 varbinds -> tooBig with empty list')
    c.ok(r_neg in rs and rs[r_neg]['vbs'][0][2] == b'afdx-es', 'negative request-id echoed')
    c.ok(r_big in rs and rs[r_big]['vbs'][0][1] == TIMETICKS, '4-byte request-id echoed')
    if r_cnt in rs:
        v = vals(rs[r_cnt])
        n_in = len(q.expect) + 2
        c.ok(v[0] == n_in and v[1] == 2 and v[2] == 0, f'snmpInPkts/BadCommunity/ParseErrs {v[:3]}')
        c.ok(v[3] == out_before(frames, r_cnt), f'snmpOutPkts {v[3]} (responses + traps before)')
        c.ok(v[4] == n_in, f'mgmtRxAccepted {v[4]}')
    return c.report()


def walk_steps():
    """(起点 OID, 期望 OID 或 None=endOfMibView)：精确对象逐项推进，外加中间/越界起点。"""
    vis = [o for o, _, _ in visible_oids()]
    steps = [((1, 3, 6, 1), vis[0])] + [(a, b) for a, b in zip(vis, vis[1:])] + [(vis[-1], None)]
    steps += [((1, 3, 6, 1, 3, 664, 2), oid_of('txMsgs')), (oid_of('vlBagUs', 4), oid_of('vlLmax', 1)),
              (oid_of('vlBagUs', 9), oid_of('vlLmax', 1)), (oid_of('vlId')[:-1], oid_of('vlId', 1)),
              (oid_of('sysDescr')[:-1], oid_of('sysDescr')), ((1, 3, 6, 1, 2, 1, 1, 4), oid_of('sysName')),
              ((1, 3, 6, 1, 3, 664, 99), None), ((1, 3, 6, 1, 6), None), ((0, 0), vis[0])]
    return steps


def scenario_snmp_walk():
    s = Script()
    q = Snmp(s)
    steps = walk_steps()
    rids = [q.req(GETNEXT, [(a, b'\x05\x00')]) for a, _ in steps]
    frames, _, _ = run(s, 'snmp_walk')
    c = Checker('snmp_walk')
    rs = responses(frames)
    c.ok(len(rs) == len(steps), f'{len(steps)} GETNEXT responses, got {len(rs)}')
    bad = []
    for rid, (start, want) in zip(rids, steps):
        r = rs.get(rid)
        if not r or r['es'] or len(r['vbs']) != 1:
            bad.append((start, 'no/err response'))
            continue
        oid, tag, _ = r['vbs'][0]
        if want is None:
            if tag != END_OF_MIB or oid != start:
                bad.append((start, oid, tag))
        elif oid != want or tag in (NO_SUCH_OBJECT, NO_SUCH_INSTANCE, END_OF_MIB):
            bad.append((start, oid, want))
    c.ok(not bad, f'GETNEXT order mismatches: {bad[:4]}')
    walked = [rs[r]['vbs'][0][0] for r in rids[:len(visible_oids())] if r in rs]
    c.ok(walked == sorted(set(walked)), 'walk strictly increasing without duplicates')
    return c.report()


def scenario_snmp_set():
    s = Script()
    q = Snmp(s)
    r_ro = q.set([('trapIntervalMs', g32(500))], comm=b'public')
    r_type = q.set([('trapIntervalMs', g32(500)), ('snmpPort', tlv(OCTETS, b'x'))])
    r_nw = q.set([('trapIntervalMs', g32(500)), ('sysDescr', tlv(OCTETS, b'x'))])
    r_nc = q.req(SET, [(oid_of('trapIntervalMs'), g32(500)), (PRIV + (9, 9, 0), g32(1))], WCOMM)
    r_val = q.set([('statsClear', integer(2))])
    r_bag = q.set([(('vlShadowBagUs', 2), g32(3000))])
    r_net = q.set([(('vlShadowNetEn', 2), integer(0))])
    r_lmax = q.set([(('vlShadowLmax', 2), g32(1472))])
    r_enc = q.set([('trapIntervalMs', tlv(GAUGE32, b'\x00\x01'))])
    r_len = q.set([('rtcSet', tlv(OCTETS, b'\x01\x02\x03'))])
    r_mix = q.set([(('vlShadowBagUs', 2), g32(8000)), ('cfgCommand', integer(1))])
    r_chk = q.get('trapIntervalMs', 'snmpPort', ('vlShadowBagUs', 2), 'cfgGeneration')
    r_ok = q.set([('trapIntervalMs', g32(5)), ('errThreshold', g32(8))])
    # statsClear=1：计数类统计改为相对清零时刻（基线由硬件逐项采样）
    r_st0 = q.get('rxDelivered', 'rxAccepted')
    r_clr = q.set([('statsClear', integer(1))])
    r_st1 = q.get('rxDelivered', 'rxAccepted')
    # Lmax 超过 ROM 硬件上限：SET 本身合法，提交校验失败 -> inconsistentValue，活动配置不变
    r_rom = q.set([(('vlShadowLmax', 2), g32(201))])
    r_rom_cmd = q.set([('cfgCommand', integer(1))])
    r_rom_chk = q.get(('vlLmax', 2), 'cfgGeneration', 'cfgLastResult')
    r_reload = q.set([('cfgCommand', integer(3))])
    r_sh = q.set([(('vlShadowBagUs', 2), g32(8000)), (('vlShadowLmax', 2), g32(150))])
    r_sh_get = q.get(('vlShadowBagUs', 2), ('vlShadowLmax', 2), ('vlBagUs', 2), ('vlLmax', 2))
    r_commit = q.set([('cfgCommand', integer(1))])
    s.wait(200)
    r_act = q.get(('vlBagUs', 2), ('vlLmax', 2), 'cfgGeneration', 'cfgLastResult', 'cfgActiveBank',
                  'trapIntervalMs', 'errThreshold')
    r_roll = q.set([('cfgCommand', integer(2))])
    s.wait(200)
    r_back = q.get(('vlBagUs', 2), ('vlLmax', 2), 'cfgGeneration', 'cfgLastResult')
    r_roll2 = q.set([('cfgCommand', integer(2))])
    # SNMP 端口：响应仍从旧端口发出，随后新端口生效、旧端口请求被丢弃
    r_port = q.set([('snmpPort', integer(1161))])
    q.get('sysName', reply=False)
    r_new = q.get('mgmtRxDropPort', 'snmpPort', port=1161)
    s.wait(8000)                    # 回滚产生的 cfgChange 在 5 ms 限速窗口后发出
    frames, _, _ = run(s, 'snmp_set')
    c = Checker('snmp_set')
    rs = responses(frames, strict=False)
    c.ok(len(rs) == len(q.expect), f'{len(q.expect)} responses, got {len(rs)}')

    def es(rid, want_es, want_ei, what):
        r = rs.get(rid)
        c.ok(r is not None and (r['es'], r['ei']) == (want_es, want_ei),
             f'{what}: want ({want_es},{want_ei}) got {(r["es"], r["ei"]) if r else None}')
    es(r_ro, 6, 1, 'read community SET -> noAccess')
    es(r_type, 7, 2, 'wrongType on 2nd varbind')
    es(r_nw, 17, 2, 'notWritable on 2nd varbind')
    es(r_nc, 11, 2, 'noCreation for unknown OID')
    es(r_val, 10, 1, 'statsClear=2 -> wrongValue')
    es(r_bag, 10, 1, 'BAG 3000 us -> wrongValue')
    es(r_net, 10, 1, 'net enable 0 -> wrongValue')
    es(r_lmax, 10, 1, 'Lmax 1472 -> wrongValue')
    es(r_enc, 9, 1, 'non-minimal Gauge32 -> wrongEncoding')
    es(r_len, 8, 1, 'rtcSet 3 bytes -> wrongLength')
    es(r_mix, 12, 2, 'cfgCommand mixed with VL writes -> inconsistentValue')
    if r_ro in rs:
        c.ok(rs[r_ro]['vbs'] == [(oid_of('trapIntervalMs'), GAUGE32, 500)], 'failed SET echoes request varbinds')
    if r_chk in rs:
        c.ok(vals(rs[r_chk]) == [1000, 161, 2000, 0], f'failed SETs changed nothing {vals(rs[r_chk])}')
    es(r_ok, 0, 0, 'two-varbind SET ok')
    es(r_clr, 0, 0, 'statsClear=1 ok')
    if r_st0 in rs and r_st1 in rs:
        before, after = vals(rs[r_st0]), vals(rs[r_st1])
        c.ok(min(before) >= 10 and all(1 <= a <= 3 for a in after),
             f'statsClear restarts counters: before {before} after {after}')
    es(r_rom, 0, 0, 'shadow Lmax 201 accepted into shadow')
    es(r_rom_cmd, 12, 1, 'commit with Lmax above ROM limit -> inconsistentValue')
    if r_rom_chk in rs:
        c.ok(vals(rs[r_rom_chk]) == [200, 0, 2], f'failed commit keeps active, cfgLastResult=2 {vals(rs[r_rom_chk])}')
    es(r_reload, 0, 0, 'cfgCommand=3 reloads shadow from active')
    es(r_sh, 0, 0, 'shadow VL SET ok')
    if r_sh_get in rs:
        c.ok(vals(rs[r_sh_get]) == [8000, 150, 2000, 200], f'shadow written, active unchanged {vals(rs[r_sh_get])}')
    es(r_commit, 0, 0, 'cfgCommand=1 commit')
    if r_act in rs:
        v = vals(rs[r_act])
        c.ok(v[:4] == [8000, 150, 1, 0] and v[5:] == [5, 8], f'committed config active {v}')
    es(r_roll, 0, 0, 'cfgCommand=2 rollback')
    if r_back in rs:
        c.ok(vals(rs[r_back]) == [2000, 200, 0, 5], f'rollback restores previous bank and generation {vals(rs[r_back])}')
    es(r_roll2, 12, 1, 'second rollback without previous bank -> inconsistentValue')
    es(r_port, 0, 0, 'snmpPort SET acknowledged from old port')
    r = rs.get(r_new)
    c.ok(r is not None and vals(r)[1] == 1161 and vals(r)[0] >= 1, 'new SNMP port serves, old port dropped')
    c.ok(all(f['sport'] == 1161 for f in frames if f['dport'] == MGR_PORT and f['start'] >
             max(x['start'] for x in frames if x['dport'] == MGR_PORT) - 1), 'reply source follows the new port')
    traps = [parse_snmp(f['payload']) for f in frames if f['dport'] == 162]
    c.ok(all(t['pdu'] == TRAP2 and t['community'] == b'public' for t in traps), 'traps are SNMPv2-Trap PDUs')
    traps = [t for t in traps if len(t['vbs']) > 1 and t['vbs'][1][2] == PRIV + (7, 6)]
    c.ok(len(traps) == 2, f'cfgChange traps on commit/rollback ({len(traps)})')
    if len(traps) == 2:
        c.ok(traps[1]['vbs'][2][2] == 1 and traps[1]['vbs'][4][2] - traps[0]['vbs'][4][2] >= 0,
             'rate-limited second trap carries its own event record')
    if traps:
        t = traps[0]
        c.ok([o for o, _, _ in t['vbs']][:2] == [oid_of('sysUpTime'), (1, 3, 6, 1, 6, 3, 1, 1, 4, 1, 0)]
             and t['vbs'][0][1] == TIMETICKS, 'trap starts with sysUpTime.0, snmpTrapOID.0')
        c.ok([o for o, _, _ in t['vbs']][2:] == [oid_of('trapEventCount'), oid_of('trapEventFirst'),
                                                  oid_of('trapEventLast')], 'trap event varbinds')
    return c.report()


def malformed():
    good = snmp_request(GET, 5, [(oid_of('sysName'), b'\x05\x00')])
    out = [good[:-1], b'\x30\x80' + good[2:] + b'\x00\x00', good + b'\x00',
           snmp_request(GET, 5, [(oid_of('sysName'), b'\x05\x00')], version=0),
           snmp_request(GET, 5, [(oid_of('sysName'), b'\x05\x00')], version=3),
           snmp_request(0xA5, 5, [(oid_of('sysName'), b'\x05\x00')]),
           snmp_request(TRAP2, 5, [(oid_of('sysName'), b'\x05\x00')]),
           bytes([0x30, len(good) - 1]) + good[2:] + b'\x00',                  # 非最短长度见下
           b'\x30\x81' + bytes([len(good) - 2]) + good[2:],
           bytes.fromhex('deadbeef00112233'), b'\x30',
           tlv(SEQ, tlv(INTEGER, b'\x00\x01') + tlv(OCTETS, b'public') + good[2 + 3 + 8:])]
    # 请求内 OID 子标识非最短（0x80 前导）
    oid = enc_oid(oid_of('sysName'))
    bad_oid = oid[:6] + b'\x80' + oid[6:]
    vbl = tlv(SEQ, tlv(SEQ, tlv(OID, bad_oid) + b'\x05\x00'))
    body = tlv(INTEGER, b'\x05') + tlv(INTEGER, b'\x00') + tlv(INTEGER, b'\x00') + vbl
    out.append(tlv(SEQ, tlv(INTEGER, b'\x01') + tlv(OCTETS, b'public') + tlv(GET, body)))
    # VarBind 长度越过父 SEQUENCE
    vb = bytearray(good)
    i = vb.index(bytes([SEQ, len(tlv(OID, enc_oid(oid_of('sysName')))) + 2]))
    vb[i + 1] += 3
    out.append(bytes(vb))
    return out


def scenario_snmp_malformed():
    s = Script()
    q = Snmp(s)
    bad = malformed()
    for m in bad:
        s.send(m, 161)
        s.wait(300)
    r_ok = q.get('sysName')
    r_cnt = q.get('snmpInParseErrs', 'snmpInPkts', 'snmpOutPkts', 'snmpInBadCommunity')
    frames, _, _ = run(s, 'snmp_malformed')
    c = Checker('snmp_malformed')
    rs = responses(frames)
    c.ok(set(rs) == {r_ok, r_cnt}, f'no response to malformed messages ({len(rs)} responses)')
    c.ok(r_ok in rs and vals(rs[r_ok]) == [b'afdx-es'], 'agent still serves after malformed input')
    if r_cnt in rs:
        v = vals(rs[r_cnt])
        c.ok(v[0] == len(bad) and v[1] == len(bad) + 2 and v[2] == out_before(frames, r_cnt) and v[3] == 0,
             f'parse error counters {v} for {len(bad)} malformed')
    return c.report()


# ---------------------------------------------------------------- TFTP / 615A
RRQ_OP, WRQ_OP, DATA_OP, ACK_OP, ERR_OP = 1, 2, 3, 4, 5
F_CFG, F_APP = b'config/afdx-vl-config.bin', b'application/app-image.bin'
F_STATUS, F_FAULT, F_ABORT = b'status/loader-status.bin', b'diagnostic/fault-log.bin', b'control/abort'
TFTP_TIMEOUT_US = 20000          # tb_afdx_mgmt_e2e 的 TFTP_TIMEOUT_US
TFTP_RETRIES = 5


def image(obj, gen, payload, crc=None, **over):
    """615A 对象清单头 + 载荷；over 可覆盖 magic/version/type/hlen/plen/flags 以构造错误。"""
    h = dict(magic=b'AFLD', version=1, type=obj, hlen=24, plen=len(payload), flags=0)
    h.update(over)
    crc = zlib.crc32(payload) if crc is None else crc
    return (h['magic'] + bytes([h['version'], h['type']]) + struct.pack('!HIIII', h['hlen'], gen, h['plen'],
                                                                         crc, h['flags']) + payload)


def cfg_payload(entries):
    """entries: [(vl, bag_code, net_en, lmax)]"""
    return b''.join(struct.pack('!HBBHH', vl, code, net, lmax, 0) for vl, code, net, lmax in entries)


def parse_tftp(p):
    op = int.from_bytes(p[:2], 'big')
    if op in (DATA_OP, ACK_OP):
        return {'op': op, 'blk': int.from_bytes(p[2:4], 'big'), 'data': p[4:]}
    if op == ERR_OP:
        if p[-1] != 0:
            raise ValueError('ERROR message not NUL terminated')
        return {'op': op, 'code': int.from_bytes(p[2:4], 'big'), 'msg': p[4:-1].decode()}
    raise ValueError(f'unexpected TFTP opcode {op}')


class Tftp:
    """TFTP 客户端脚本：每个会话使用独立客户端端口，服务器 TID 由测试平台从首个应答学习（dport=0）。"""

    def __init__(self, script, port):
        self.s, self.port = script, port

    def req(self, op, name, mode=b'octet', extra=b'', reply=True):
        self.s.send(struct.pack('!H', op) + name + b'\0' + mode + b'\0' + extra, 69, self.port)
        if reply:
            self.s.recv(1)

    def data(self, blk, chunk, reply=True, tid=0):
        self.s.send(struct.pack('!HH', DATA_OP, blk) + chunk, tid, self.port)
        if reply:
            self.s.recv(1)

    def ack(self, blk, reply=True, tid=0):
        self.s.send(struct.pack('!HH', ACK_OP, blk), tid, self.port)
        if reply:
            self.s.recv(1)

    def put(self, name, blob, reply_last=True):
        """完整 WRQ：WRQ→ACK0，逐块 DATA→ACK，长度为 512 整数倍时补 0 字节块。"""
        self.req(WRQ_OP, name)
        blocks = [blob[i:i + 512] for i in range(0, len(blob), 512)]
        if len(blob) % 512 == 0:
            blocks.append(b'')
        for i, b in enumerate(blocks, 1):
            self.data(i, b, reply=reply_last or i < len(blocks))
        return len(blocks)

    def get(self, name, size):
        """完整 RRQ：逐块 ACK，size 为期望对象长度。"""
        self.req(RRQ_OP, name)
        n = size // 512 + 1
        for i in range(1, n + 1):
            self.ack(i, reply=i < n)
        return n


def tftp_frames(frames, port):
    return [dict(parse_tftp(f['payload']), sport=f['sport'], start=f['start'])
            for f in frames if f['dport'] == port]


def assemble(pkts):
    out, blk = b'', 0
    for p in pkts:
        if p['op'] == DATA_OP and p['blk'] == blk + 1:
            out += p['data']
            blk += 1
    return out


def check_image(c, blob, obj, what):
    ok = len(blob) >= 24 and blob[:4] == b'AFLD' and blob[4] == 1 and blob[5] == obj
    if ok:
        _, gen, plen, crc, flags = struct.unpack('!HIIII', blob[6:24])
        ok = plen == len(blob) - 24 and zlib.crc32(blob[24:]) == crc and flags == 0
    c.ok(ok, f'{what}: well-formed image with valid CRC')
    return struct.unpack('!I', blob[8:12])[0] if ok else None


STATUS_FIELDS = ('state', 'object', 'error', 'app_bank', 'fail_block', 'cur_block', 'bytes', 'crc_expected',
                 'crc_actual', 'cfg_generation', 'app_version', 'app_length', 'loads', 'unloads', 'app_crc')


def parse_status(blob):
    if len(blob) != 48 or blob[:4] != b'AFST':
        raise ValueError(f'bad status object {blob[:8]!r} len {len(blob)}')
    return dict(zip(STATUS_FIELDS, struct.unpack('!BBBBHHIIIIIIIII', blob[4:])))


def scenario_tftp_basic():
    s = Script()
    p = {k: 40000 + i for i, k in enumerate(('status', 'nofile', 'mode', 'wstat', 'rctl', 'cfg', 'app', 'op',
                                             'trav', 'dsl', 'trunc', 'opts', 'tid', 'fault'))}
    t = {k: Tftp(s, v) for k, v in p.items()}
    t['status'].req(RRQ_OP, F_STATUS)
    t['status'].ack(1, reply=False)
    t['nofile'].req(RRQ_OP, b'foo.bin')
    t['mode'].req(RRQ_OP, F_STATUS, mode=b'netascii')
    t['wstat'].req(WRQ_OP, F_STATUS)
    t['rctl'].req(RRQ_OP, F_ABORT)
    t['cfg'].req(RRQ_OP, F_CFG, mode=b'OcTeT')
    t['cfg'].ack(1, reply=False)
    t['app'].req(RRQ_OP, F_APP)
    s.send(struct.pack('!H', 6) + F_CFG + b'\0octet\0', 69, p['op'])
    s.recv(1)
    t['trav'].req(RRQ_OP, b'../' + F_CFG)
    t['dsl'].req(RRQ_OP, b'config//afdx-vl-config.bin')
    s.send(struct.pack('!H', RRQ_OP) + F_STATUS, 69, p['trunc'])       # 无 NUL 结尾
    s.recv(1)
    t['opts'].req(RRQ_OP, F_STATUS, extra=b'blksize\x001024\x00')
    t['tid'].req(RRQ_OP, F_FAULT)
    s.send(struct.pack('!HH', ACK_OP, 1), 0xC00F, p['tid'])             # 未知 TID
    s.recv(1)
    t['tid'].ack(1, reply=False)
    t['opts'].ack(1, reply=False)
    q = Snmp(s)
    r_cnt = q.get('tftpErrors', 'tftpRetries', 'ldUnloads')
    frames, _, _ = run(s, 'tftp_basic')
    c = Checker('tftp_basic')
    tf = {k: tftp_frames(frames, v) for k, v in p.items()}

    def err(k, code, what):
        x = tf[k]
        c.ok(len(x) == 1 and x[0]['op'] == ERR_OP and x[0]['code'] == code and x[0]['sport'] == 69,
             f'{what}: ERROR {code} from port 69, got {x}')
    st = tf['status']
    c.ok(len(st) == 1 and st[0]['op'] == DATA_OP and st[0]['blk'] == 1 and 0xC000 <= st[0]['sport'] <= 0xC00F,
         'RRQ status -> DATA 1 from a server TID')
    if st:
        try:
            v = parse_status(st[0]['data'])
            c.ok(v['state'] == 0 and v['cfg_generation'] == 0 and v['loads'] == 0, f'initial status {v}')
        except ValueError as e:
            c.ok(False, str(e))
    err('nofile', 1, 'unknown file')
    err('mode', 0, 'netascii mode')
    err('wstat', 2, 'WRQ to read-only status')
    err('rctl', 2, 'RRQ of control object')
    err('app', 1, 'RRQ of absent application image')
    err('op', 4, 'unsupported opcode')
    err('trav', 1, 'path traversal')
    err('dsl', 1, 'doubled separator')
    err('trunc', 4, 'unterminated file name')
    cf = tf['cfg']
    c.ok(len(cf) == 1 and cf[0]['op'] == DATA_OP and len(cf[0]['data']) == 24 + 8 * N_TX_VL,
         'case-insensitive octet mode; config unload fits one block')
    if cf:
        blob = cf[0]['data']
        check_image(c, blob, 1, 'config unload')
        ent = [struct.unpack('!HBBHH', blob[24 + 8 * i: 32 + 8 * i]) for i in range(N_TX_VL)]
        c.ok(ent == [(i, i, VL_ROM['vlNetEn'][i], VL_ROM['vlLmax'][i], 0) for i in range(N_TX_VL)],
             f'config unload matches active configuration {ent}')
    c.ok(len(tf['opts']) == 1 and tf['opts'][0]['op'] == DATA_OP, 'RFC 2347 options ignored (no OACK)')
    x = tf['tid']
    c.ok(len(x) == 2 and x[0]['op'] == DATA_OP and x[1]['op'] == ERR_OP and x[1]['code'] == 5
         and x[1]['sport'] == 0xC00F, f'unknown TID -> ERROR 5 from that port {x}')
    if x:
        c.ok(x[0]['data'][:4] == b'AFFL' and len(x[0]['data']) == 64, 'fault log object')
    extra = [f for f in frames if f['dport'] in p.values()]
    c.ok(len(extra) == sum(len(v) for v in tf.values()), 'no unexpected TFTP packets')
    r = responses(frames).get(r_cnt)
    c.ok(r is not None and vals(r) == [10, 0, 4], f'tftpErrors/tftpRetries/ldUnloads {vals(r) if r else None}')
    return c.report()


def gaps(frames, rids):
    """同一管理 VL 上连续响应帧的 SOF 间隔（us）。"""
    starts = [f['start'] for f in frames if f['dport'] == MGR_PORT and f['sport'] == 161
              and parse_snmp(f['payload'], False)['rid'] in rids]
    return [(b - a) / 1000 for a, b in zip(starts, starts[1:])]


def scenario_tftp_config():
    s = Script()
    q = Snmp(s)
    before = [q.get('sysUpTime') for _ in range(3)]
    new_cfg = [(0, 3, 3, 1471), (1, 0, 1, 100), (3, 1, 2, 64)]
    t1 = Tftp(s, 41000)
    t1.put(F_CFG, image(1, 5, cfg_payload(new_cfg)))
    s.wait(6000)                                    # 自检窗口（SELFTEST_US=5000）
    r_act = q.get(('vlBagUs', 1), ('vlBagUs', 2), ('vlLmax', 2), ('vlNetEn', 2), ('vlBagUs', 3),
                  ('vlBagUs', 4), ('vlNetEn', 4), 'ldCfgVersion', 'cfgGeneration', 'ldState', 'ldLoads')
    after = [q.get('sysUpTime') for _ in range(3)]
    t2 = Tftp(s, 41001)
    n_rd = t2.get(F_CFG, 24 + 8 * N_TX_VL)
    t3 = Tftp(s, 41002)                              # generation 不增
    t3.put(F_CFG, image(1, 5, cfg_payload(new_cfg)))
    t4 = Tftp(s, 41003)                              # CRC 错误
    t4.put(F_CFG, image(1, 6, cfg_payload(new_cfg), crc=0x12345678))
    r_crc = q.get('ldState', 'ldLastError', 'ldCrcExpected', 'ldCrcActual', ('vlBagUs', 1), 'cfgGeneration')
    bad = [(t, image(1, 6, cfg_payload(e), **o)) for t, e, o in (
        (41004, [(0, 9, 3, 100)], {}), (41005, [(1, 0, 3, 100), (1, 0, 3, 100)], {}),
        (41006, [(0, 0, 0, 100)], {}), (41007, [(0, 0, 3, 1472)], {}), (41008, [(7, 0, 3, 100)], {}),
        (41009, [(0, 0, 3, 100)], {'magic': b'XFLD'}), (41010, [(0, 0, 3, 100)], {'type': 2}),
        (41011, [(0, 0, 3, 100)], {'version': 2}), (41012, [(0, 0, 3, 100)], {'plen': 16}))]
    for port, blob in bad:
        Tftp(s, port).put(F_CFG, blob)
    t5 = Tftp(s, 41013)                              # 合法但带宽/校验失败：Lmax 超过 ROM 上限
    t5.put(F_CFG, image(1, 7, cfg_payload([(2, 0, 1, 65)])))
    r_end = q.get(('vlBagUs', 1), ('vlLmax', 3), 'cfgGeneration', 'ldState', 'ldLastError', 'ldLoads')
    frames, _, _ = run(s, 'tftp_config')
    c = Checker('tftp_config')
    rs = responses(frames)
    g0, g1 = gaps(frames, before), gaps(frames, after)
    c.ok(len(g0) == 2 and all(900 <= g <= 1500 for g in g0), f'mgmt VL paced by 1 ms BAG before load {g0}')
    c.ok(len(g1) == 2 and all(g >= 8000 for g in g1), f'mgmt VL paced by 8 ms BAG after load {g1}')
    w = tftp_frames(frames, 41000)
    c.ok([(x['op'], x['blk']) for x in w] == [(ACK_OP, 0), (ACK_OP, 1)], f'WRQ config ACK 0/1 {w}')
    if r_act in rs:
        v = vals(rs[r_act])
        c.ok(v == [8000, 1000, 100, 1, 4000, 2000, 2, 5, 5, 5, 1], f'loaded config active {v}')
    blob = assemble(tftp_frames(frames, 41001))
    c.ok(n_rd == 1 and check_image(c, blob, 1, 'config readback') == 5, 'readback generation 5')
    if len(blob) == 24 + 8 * N_TX_VL:
        ent = [struct.unpack('!HBBHH', blob[24 + 8 * i: 32 + 8 * i]) for i in range(N_TX_VL)]
        c.ok(ent == [(0, 3, 3, 1471, 0), (1, 0, 1, 100, 0), (2, 2, 1, 64, 0), (3, 1, 2, 64, 0)],
             f'readback reflects loaded + untouched entries {ent}')

    def load_err(port, code, what):
        x = tftp_frames(frames, port)
        e = [y for y in x if y['op'] == ERR_OP]
        c.ok(len(e) == 1 and e[0]['msg'] == f'LOAD {code:02X}', f'{what}: ERROR "LOAD {code:02X}" got {x}')
    load_err(41002, 7, 'generation not increasing')
    load_err(41003, 6, 'payload CRC mismatch')
    if r_crc in rs:
        v = vals(rs[r_crc])
        c.ok(v[0] == 6 and v[1] == 6 and v[2] == 0x12345678 and v[3] == zlib.crc32(cfg_payload(new_cfg))
             and v[4:] == [8000, 5], f'CRC failure keeps active bank {v}')
    for (port, _), code, what in zip(bad, (10, 9, 11, 12, 8, 1, 3, 2, 5), (
            'BAG code', 'VL order', 'network enable', 'Lmax', 'VL index', 'magic', 'object type', 'format version',
            'payload length')):
        load_err(port, code, what)
    load_err(41013, 13, 'runtime verification (Lmax above ROM)')
    if r_end in rs:
        c.ok(vals(rs[r_end]) == [8000, 64, 5, 6, 13, 1], f'failed loads leave generation 5 active {vals(rs[r_end])}')
    return c.report()


def scenario_tftp_app():
    s = Script()
    q = Snmp(s)
    pay = bytes((i * 7 + 3) & 0xFF for i in range(1000))
    blob = image(2, 2, pay)                          # 1024 字节：两整块 + 0 字节结束块
    t = Tftp(s, 42000)
    t.req(WRQ_OP, F_APP)
    t.req(WRQ_OP, F_APP)                             # 重复 WRQ：重发 ACK 0，不另开会话
    t.data(1, blob[:512])
    t.data(1, blob[:512])                            # 重复 DATA：只重发 ACK，不重复写入
    s.send(struct.pack('!HH', DATA_OP, 2) + blob[512:], 0xC00E, 42000)   # 错误 TID
    s.recv(1)
    Tftp(s, 42009).data(2, blob[512:], tid=-42000)   # 同一 TID、源端口变化 -> ERROR 5
    t.data(2, blob[512:])
    t.data(3, b'')
    t.data(3, b'')                                   # 最后 ACK 丢失：重发 ACK 3
    r_st = q.get('ldState', 'ldAppBank', 'ldAppLength', 'ldAppVersion', 'ldAppCrc', 'ldLoads')
    n = Tftp(s, 42001).get(F_APP, len(blob))
    # 第二次加载中途出现未来块：终止，活动银行保持
    t2 = Tftp(s, 42002)
    t2.req(WRQ_OP, F_APP)
    t2.data(1, image(2, 3, pay)[:512])
    t2.data(3, b'x' * 10)
    r_mid = q.get('ldState', 'ldAppBank', 'ldAppVersion')
    # control/abort 终止进行中的写会话（先等 42000 写会话结束最后 ACK 的停留期，释放会话槽）
    s.wait(TFTP_TIMEOUT_US + 3000)
    t3 = Tftp(s, 42003)
    t3.req(WRQ_OP, F_APP)
    t3.data(1, image(2, 4, pay)[:512])
    t4 = Tftp(s, 42004)
    t4.req(WRQ_OP, F_ABORT)
    t4.data(1, b'ABRT')
    t3.data(2, image(2, 4, pay)[512:])
    s.wait(TFTP_TIMEOUT_US + 3000)                   # 控制会话最后 ACK 的停留期
    t7 = Tftp(s, 42007)                              # 控制记录不是 ABRT：ERROR 并立即释放会话
    t7.req(WRQ_OP, F_ABORT)
    t7.data(1, b'STOP')
    t6 = Tftp(s, 42006)
    t6.req(WRQ_OP, F_APP)
    t5 = Tftp(s, 42005)                              # 暂存中另一暂存写入 -> BUSY
    t5.req(WRQ_OP, F_CFG)
    r_ab = q.set([('ldAbort', integer(1))])
    t6.data(1, b'y' * 100)
    r_end = q.get('ldState', 'ldLastError', 'ldAppBank', 'ldAppVersion', 'ldLoads')
    frames, _, _ = run(s, 'tftp_app')
    c = Checker('tftp_app')
    rs = responses(frames)
    w = [x for x in tftp_frames(frames, 42000) if x['sport'] != 0xC00E]
    c.ok([(x['op'], x.get('blk')) for x in w] == [(ACK_OP, 0), (ACK_OP, 0), (ACK_OP, 1), (ACK_OP, 1),
                                                  (ACK_OP, 2), (ACK_OP, 3), (ACK_OP, 3)], f'WRQ sequence {w}')
    c.ok(len({x['sport'] for x in w}) == 1, 'duplicate WRQ reuses the session TID')
    e = [x for x in frames if x['sport'] == 0xC00E]
    c.ok(len(e) == 1 and parse_tftp(e[0]['payload'])['code'] == 5 and e[0]['dport'] == 42000, 'wrong TID -> ERROR 5')
    e = tftp_frames(frames, 42009)
    c.ok(len(e) == 1 and e[0]['op'] == ERR_OP and e[0]['code'] == 5, f'changed client port -> ERROR 5 {e}')
    if r_st in rs:
        c.ok(vals(rs[r_st]) == [5, 1, 1000, 2, zlib.crc32(pay), 1], f'application image active {vals(rs[r_st])}')
    rb = tftp_frames(frames, 42001)
    c.ok(n == 3 and [len(x['data']) for x in rb] == [512, 512, 0], f'RRQ image blocks {[len(x["data"]) for x in rb]}')
    c.ok(assemble(rb) == blob, 'unloaded image identical to loaded image (no double write)')
    x = tftp_frames(frames, 42002)
    c.ok([y['op'] for y in x] == [ACK_OP, ACK_OP, ERR_OP] and x[-1]['code'] == 4, f'future block -> ERROR 4 {x}')
    if r_mid in rs:
        c.ok(vals(rs[r_mid]) == [7, 1, 2], f'aborted load keeps active image {vals(rs[r_mid])}')
    x = tftp_frames(frames, 42003)
    c.ok([y['op'] for y in x] == [ACK_OP, ACK_OP, ERR_OP] and x[-1]['code'] == 5,
         f'write session released by control/abort {x}')
    x = tftp_frames(frames, 42004)
    c.ok([(y['op'], y.get('blk')) for y in x] == [(ACK_OP, 0), (ACK_OP, 1)], f'control/abort accepted {x}')
    x = tftp_frames(frames, 42005)
    c.ok(len(x) == 1 and x[0]['op'] == ERR_OP and x[0]['code'] == 0, f'second staging write -> BUSY {x}')
    x = tftp_frames(frames, 42007)
    c.ok([y['op'] for y in x] == [ACK_OP, ERR_OP], f'bad control record rejected {x}')
    c.ok(r_ab in rs and rs[r_ab]['es'] == 0, 'SNMP ldAbort accepted')
    x = tftp_frames(frames, 42006)
    c.ok([y['op'] for y in x] == [ACK_OP, ERR_OP] and x[-1]['code'] == 5, f'SNMP abort released session {x}')
    if r_end in rs:
        c.ok(vals(rs[r_end]) == [7, 15, 1, 2, 1], f'aborts keep image generation 2 {vals(rs[r_end])}')
    return c.report()


def scenario_tftp_timeout():
    s = Script()
    q = Snmp(s)
    t = Tftp(s, 43000)
    t.req(RRQ_OP, F_STATUS)
    s.wait((TFTP_RETRIES + 2) * (TFTP_TIMEOUT_US + 2000))
    t.ack(1)                                         # 会话已放弃 -> ERROR 5
    w = Tftp(s, 43001)
    w.req(WRQ_OP, F_APP)
    w.data(1, image(2, 1, bytes(600))[:512])
    s.wait((TFTP_RETRIES + 2) * (TFTP_TIMEOUT_US + 2000))
    r = q.get('tftpRetries', 'ldState', 'ldLastError', 'ldAppVersion')
    frames, _, _ = run(s, 'tftp_timeout')
    c = Checker('tftp_timeout')
    x = tftp_frames(frames, 43000)
    c.ok([y['op'] for y in x] == [DATA_OP] * (TFTP_RETRIES + 1) + [ERR_OP] and all(
        y.get('blk') == 1 for y in x[:-1]), f'DATA 1 retransmitted {TFTP_RETRIES} times then dropped {[y["op"] for y in x]}')
    iv = [(b['start'] - a['start']) / 1000 for a, b in zip(x, x[1:TFTP_RETRIES + 1])]
    # 定时从报文入队开始，首帧 SOF 另受 BAG 延迟，故首个间隔可略短于超时值
    c.ok(len(iv) == TFTP_RETRIES and all(TFTP_TIMEOUT_US - 1500 <= v <= TFTP_TIMEOUT_US + 3000 for v in iv),
         f'retransmit interval {iv}')
    x = tftp_frames(frames, 43001)
    c.ok([(y['op'], y.get('blk')) for y in x] == [(ACK_OP, 0)] + [(ACK_OP, 1)] * (TFTP_RETRIES + 1),
         f'last ACK retransmitted on write timeout {[(y["op"], y.get("blk")) for y in x]}')
    rs = responses(frames)
    c.ok(r in rs and vals(rs[r]) == [2 * TFTP_RETRIES, 7, 15, 0], f'retry exhaustion aborts staging {vals(rs[r]) if r in rs else None}')
    return c.report()


# ---------------------------------------------------------------- RTC
RTC_STATES = ('UNSYNCED', 'ACQUIRING', 'LOCKED', 'HOLDOVER', 'DEGRADED', 'FAULT')


def time_msgs(frames):
    out = []
    for f in frames:
        p = f['payload']
        if f['sport'] == RTC_PORT and len(p) == 36 and p[:4] == b'ATIM':
            out.append({'type': p[5], 'seq': int.from_bytes(p[8:12], 'big'), 'src': int.from_bytes(p[12:14], 'big'),
                        'state': p[14], 'ts': int.from_bytes(p[16:24], 'big'), 'start': f['start'],
                        'dport': f['dport']})
    return out


def scenario_rtc_lock():
    s = Script()
    q = Snmp(s)
    # 主站快 100 ppm、领先 123.456 ms；周期 2^14 us。时间戳分辨率 1 us，频率积分的
    # 最小步长为 1 us / 周期 / 4 ≈ 15 ppm，故频率只能在一个量化步内跟踪。
    n, logp, drift_ppm = 30, 14, 100
    period = 1 << logp
    s.master(drift_ppm * 1000, 123456)
    s.auto_resp(True)
    for i in range(n):
        s.sync(1000 + i, logp=logp)
        s.wait(400)                                  # DELAY_RESP 与伺服
        s.sample()
        s.wait(period - 1400)
    r = q.get('rtcState', 'rtcSyncs', 'rtcSteps', 'rtcLastSeq', 'rtcSourceId', 'rtcFreqCorr', 'rtcPathDelay',
              'rtcLastOffset', 'rtcMaxAbsOffset', 'rtcLost', 'rtcTimeouts', 'rtcTime')
    frames, _, samples = run(s, 'rtc_lock')
    c = Checker('rtc_lock')
    reqs = time_msgs(frames)
    c.ok(len(reqs) == n and all(m['type'] == 2 and m['dport'] == RTC_PORT for m in reqs)
         and [m['seq'] for m in reqs] == list(range(1000, 1000 + n)), f'{n} DELAY_REQ echo SYNC sequence')
    err = [x['rtc'] - x['master'] for x in samples]
    st = [x['state'] for x in samples]
    c.ok(len(samples) == n and abs(err[0]) < 50 and st[0] == 1, f'first SYNC steps RTC (err {err[:3]}, state {st[:3]})')
    lock_at = next((i for i, v in enumerate(st) if v == 2), None)
    c.ok(lock_at is not None and all(v == 2 for v in st[lock_at:]), f'reaches and holds LOCKED (states {st})')
    tail = err[-12:]
    c.ok(max(map(abs, tail)) <= 5, f'steady-state |RTC - master| <= 5 us under {drift_ppm} ppm drift: {tail}')
    rs = responses(frames)
    if r in rs:
        v = vals(rs[r])
        ppm = v[5] / 2 ** 32 * 1e6
        c.ok(v[0] == 2 and v[1] == n and v[2] == 1 and v[3] == 1000 + n - 1 and v[4] == 0x0A01,
             f'SNMP state/syncs/steps/lastSeq/source {v[:5]}')
        c.ok(abs(ppm - drift_ppm) <= 20, f'frequency correction {ppm:.1f} ppm tracks {drift_ppm} ppm drift')
        c.ok(v[6] < 30 and abs(v[7]) <= 5 and v[8] >= 123000 and v[9] == 0 and v[10] == 0,
             f'path delay {v[6]} us, last offset {v[7]}, max {v[8]}, lost/timeouts {v[9:11]}')
        fr = next(f for f in frames if f['dport'] == MGR_PORT)
        c.ok(abs(v[11] - (fr['start'] * (1 + drift_ppm / 1e6) / 1000 + 123456)) < 1500,
             'rtcTime (Counter64) matches master time')
    return c.report()


def scenario_rtc_proto():
    s = Script()
    q = Snmp(s)
    r_cfg = q.set([('rtcHoldoverUs', g32(30000)), ('rtcUnsyncUs', g32(80000))])
    s.master(0, 5000)
    s.auto_resp(True)
    for seq in (0xFFFFFFFE, 0xFFFFFFFF, 0, 1):      # 序号跨回绕
        s.sync(seq)
        s.wait(3000)
    r_wrap = q.get('rtcLost', 'rtcOutOfOrder', 'rtcSyncs', 'rtcLastSeq')
    s.dup()                                          # 同序号同内容：重复
    s.wait(1500)
    s.sync(1, reply=False)                           # 同序号不同时间戳：冲突
    s.wait(1500)
    s.sync(0xFFFFFFF0, reply=False)                  # 旧序号
    s.wait(1500)
    s.sync(6)                                        # 跳过 2..5
    s.wait(3000)
    s.sync(50, src=0x0A02, q=20, reply=False)        # 质量更差的另一源：拒绝
    s.wait(1500)
    s.sync(100, src=0x0A02, q=5)                     # 更优源：切换
    s.wait(3000)
    for seq in range(101, 106):                      # 新源上重新锁定
        s.sync(seq, src=0x0A02, q=5)
        s.wait(3000)
    r_mid = q.get('rtcDuplicate', 'rtcBadFormat', 'rtcOutOfOrder', 'rtcLost', 'rtcRejected', 'rtcSourceSwitches',
                  'rtcSourceId', 'rtcSourceQuality', 'rtcLastSeq', 'rtcSyncs', 'rtcState')
    s.auto_resp(False)
    s.sync(106, src=0x0A02, q=5)                     # DELAY_RESP 丢失
    s.wait(35000)
    s.sample()                                       # HOLDOVER
    s.wait(60000)
    s.sample()                                       # UNSYNCED
    s.wait(20000)
    s.auto_resp(True)
    s.oneway(107, src=0x0A02, q=5)
    s.wait(1500)
    s.sample()                                       # DEGRADED
    s.sync(108, src=0x0A02, q=5, status=1, reply=False)
    s.wait(1500)
    s.sample()                                       # FAULT
    r_end = q.get('rtcState', 'rtcTimeouts', 'rtcLastError', 'rtcLastSeq', 'rtcRejected', 'rtcSteps')
    # 手动设时跨 32 位边界；BAG/单调时间不受 RTC 步进影响
    r_set = q.set([('rtcSet', tlv(OCTETS, ((1 << 32) - 3000).to_bytes(8, 'big')))])
    s.wait(5000)
    r_time = q.get('rtcTime', 'rtcTimeLow', 'rtcState', 'esMonotonicUs')
    r_back = q.set([('rtcSet', tlv(OCTETS, (1000).to_bytes(8, 'big')))])
    pace = [q.get('esMonotonicUs', 'rtcTimeLow') for _ in range(3)]
    r_ro = q.set([('rtcSet', tlv(OCTETS, bytes(8)))], comm=b'public')
    frames, _, samples = run(s, 'rtc_proto')
    c = Checker('rtc_proto')
    rs = responses(frames)
    c.ok(len(rs) == len(q.expect), f'{len(q.expect)} SNMP responses, got {len(rs)}')
    c.ok(r_cfg in rs and rs[r_cfg]['es'] == 0, 'holdover/unsync thresholds written')
    if r_wrap in rs:
        c.ok(vals(rs[r_wrap]) == [0, 0, 4, 1], f'sequence wrap is not loss/out-of-order {vals(rs[r_wrap])}')
    if r_mid in rs:
        c.ok(vals(rs[r_mid]) == [1, 1, 1, 4, 1, 1, 0x0A02, 5, 105, 11, 2], f'duplicate/conflict/old/lost/source {vals(rs[r_mid])}')
    reqs = [m['seq'] for m in time_msgs(frames)]
    c.ok(reqs == [0xFFFFFFFE, 0xFFFFFFFF, 0, 1, 6] + list(range(100, 107)), f'DELAY_REQ only for accepted SYNC {reqs}')
    st = [x['state'] for x in samples]
    c.ok(st == [3, 0, 4, 5], f'HOLDOVER -> UNSYNCED -> DEGRADED (one-way) -> FAULT: {[RTC_STATES[x] for x in st]}')
    if r_end in rs:
        c.ok(vals(rs[r_end]) == [5, 2, 8, 107, 2, 2], f'state/timeouts/lastError/lastSeq/rejected/steps {vals(rs[r_end])}')
    c.ok(r_set in rs and rs[r_set]['es'] == 0, 'rtcSet accepted with write community')
    if r_time in rs:
        t64, low, stt, _ = vals(rs[r_time])
        c.ok((1 << 32) + 1000 < t64 < (1 << 32) + 4000 and 0 <= low - (t64 & 0xFFFFFFFF) < 50 and stt == 0,
             f'64-bit RTC crosses 2^32 ({t64}), low word wraps ({low}), state UNSYNCED')
    c.ok(r_back in rs and rs[r_back]['es'] == 0, 'rtcSet backwards accepted')
    p = [vals(rs[r]) for r in pace if r in rs]
    g = gaps(frames, pace)
    c.ok(len(p) == 3 and all(900 <= x <= 1500 for x in g) and
         all(abs((b[0] - a[0]) - gg) < 50 for a, b, gg in zip(p, p[1:], g)),
         f'after backward step BAG pacing and monotonic time unaffected {g} {p}')
    c.ok(len(p) == 3 and p[0][1] < 20000, f'RTC runs from the new value {p[:1]}')
    c.ok(r_ro in rs and rs[r_ro]['es'] == 6, 'rtcSet with read community -> noAccess')
    return c.report()


# ---------------------------------------------------------------- Trap
def traps_of(frames, port=162):
    out = []
    for f in frames:
        if f['dport'] == port and f['sport'] == 161:
            t = parse_snmp(f['payload'])
            ok = t['pdu'] == TRAP2 and len(t['vbs']) == 5 and t['vbs'][1][2][:-1] == PRIV + (7,)
            out.append({'ok': ok, 'type': t['vbs'][1][2][-1] if ok else None, 'uptime': t['vbs'][0][2],
                        'count': t['vbs'][2][2], 'first': t['vbs'][3][2], 'last': t['vbs'][4][2],
                        'start': f['start'], 'rid': t['rid']})
    return out


def scenario_trap():
    s = Script()
    q = Snmp(s)
    q.set([('trapIntervalMs', g32(20)), ('linkTimeoutMs', g32(5))])
    s.wait(30000)                                    # 网络 A 无帧 5 ms -> 链路中断，20 ms 窗口后上报
    q.set([('trapDstPort', integer(1162)), ('linkTimeoutMs', g32(100000))])
    bad = image(1, 9, cfg_payload([(0, 0, 3, 100)]), magic=b'XXXX')
    for i in range(3):                               # 20 ms 窗口内 3 次加载失败合并
        Tftp(s, 44000 + i).put(F_CFG, bad)
    s.wait(25000)
    q.set([('trapMask', g32(1 << 4))])
    Tftp(s, 44010).put(F_CFG, bad)                   # 被屏蔽
    s.wait(25000)
    r = q.get('trapSent', 'trapDropped', 'snmpOutPkts', 'trapEventType', 'trapMask')
    frames, _, _ = run(s, 'trap')
    c = Checker('trap')
    t162, t1162 = traps_of(frames), traps_of(frames, 1162)
    c.ok(all(t['ok'] for t in t162 + t1162), 'all traps are SNMPv2-Trap with the standard varbinds')
    c.ok([t['type'] for t in t162] == [1, 1], f'link up then link down traps on 162 {[t["type"] for t in t162]}')
    if len(t162) == 2:
        c.ok(5000 <= t162[1]['first'] - t162[0]['first'] <= 6000 and t162[1]['count'] == 1
             and t162[1]['start'] >= (t162[0]['first'] + 20000) * 1000 and t162[1]['uptime'] >= t162[0]['uptime'],
             f'link down after 5 ms silence, reported after the 20 ms window {t162}')
    ld = [t for t in t1162 if t['type'] == 4]
    c.ok(len(ld) == 2 and ld[0]['count'] == 1 and ld[1]['count'] == 2,
         f'loaderFault: first trap then one merged trap for the rest {[(t["count"]) for t in ld]}')
    if len(ld) == 2:
        # 窗口从上一条 Trap 作业开始（≈ 其事件时间）计时；帧 SOF 另受 BAG 排队影响
        c.ok(ld[1]['start'] >= (ld[0]['first'] + 20000) * 1000 and ld[1]['last'] > ld[1]['first'],
             f'merged trap waits for the rate-limit window and carries first/last times {ld}')
    c.ok(not [t for t in t1162 if t['type'] == 4 and t['start'] > ld[-1]['start'] + 30e6] if ld else True,
         'masked loader fault produces no trap')
    c.ok(len({t['rid'] for t in t162 + t1162}) == len(t162 + t1162), 'trap request-ids unique')
    rs = responses(frames)
    if r in rs:
        v = vals(rs[r])
        c.ok(v[0] == len(t162) + len(t1162) and v[1] == 1 and v[4] == 16,
             f'trapSent {v[0]} = traps captured {len(t162) + len(t1162)}, trapDropped {v[1]}')
    return c.report()


SCENARIOS = {'smoke': scenario_smoke, 'snmp_get': scenario_snmp_get, 'snmp_walk': scenario_snmp_walk,
             'snmp_set': scenario_snmp_set, 'snmp_malformed': scenario_snmp_malformed,
             'tftp_basic': scenario_tftp_basic, 'tftp_config': scenario_tftp_config,
             'tftp_app': scenario_tftp_app, 'tftp_timeout': scenario_tftp_timeout,
             'rtc_lock': scenario_rtc_lock, 'rtc_proto': scenario_rtc_proto, 'trap': scenario_trap}


def main():
    global CAPTURE
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("scenario", choices=SCENARIOS)
    ap.add_argument("capture", type=Path)
    args = ap.parse_args()
    CAPTURE = args.capture
    result = SCENARIOS[args.scenario]()
    return 1 if result["errors"] else 0

if __name__ == "__main__":
    sys.exit(main())
