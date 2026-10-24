"""独立 SNMP 测试模型：OID、类型与访问属性；不生成或修改设计文件。"""
PREFIX = {0: (1, 3, 6, 1, 2, 1, 1), 1: (1, 3, 6, 1, 3, 664), 2: (1, 3, 6, 1, 6, 3, 1, 1, 4, 1)}
TYPES = {'INTEGER': 0x02, 'OCTET': 0x04, 'OID': 0x06, 'C32': 0x41, 'G32': 0x42, 'TT': 0x43, 'C64': 0x46}
ACCESS = {'ro': 0, 'rw': 1, 'hidden': 2}
# 写入检查类别（afdx_snmp_agent 第一阶段校验）
CHECKS = {'none': 0, 'one': 1, 'cmd': 2, 'port': 3, 'any': 4, 'bag': 5, 'lmax': 6, 'net': 7,
          'oct8': 8, 'nz': 9}

# (名称, 基, 后缀弧, 类型, 访问, 检查, 说明)；表列的后缀为 (3,1,1,列)，实例为 VL 索引+1
MIB = [
    ('sysDescr', 0, (1, 0), 'OCTET', 'ro', 'none', '设备描述'),
    ('sysObjectID', 0, (2, 0), 'OID', 'ro', 'none', '= 私有根'),
    ('sysUpTime', 0, (3, 0), 'TT', 'ro', 'none', '复位后 1/100 s'),
    ('sysName', 0, (5, 0), 'OCTET', 'ro', 'none', '设备名'),
    ('esFwVersion', 1, (1, 1, 0), 'G32', 'ro', 'none', '固件版本'),
    ('esBuildId', 1, (1, 2, 0), 'G32', 'ro', 'none', '构建标识'),
    ('esIdentity', 1, (1, 3, 0), 'G32', 'ro', 'none', '网络号×256 + 设备号'),
    ('esMonotonicUs', 1, (1, 4, 0), 'G32', 'ro', 'none', '单调微秒计数'),
]
_stats = ['txMsgs', 'txFrames', 'txFramesA', 'txFramesB', 'txLmaxDrop', 'txOvfDrop', 'rxFramesA',
          'rxFramesB', 'rxFcsErr', 'rxMacLenErr', 'rxMacOvf', 'rxHdrErr', 'rxLenErr', 'rxVlUnknown',
          'rxCsumErr', 'rxFullDrop', 'rxIcDrop', 'rxRmDrop', 'rxAccepted', 'rxDelivered']
for i, name in enumerate(_stats, 1):
    MIB.append((name, 1, (2, i, 0), 'C32', 'ro', 'none', '数据面计数（相对最近一次清零）'))
MIB += [
    ('txBufUsed', 1, (2, 21, 0), 'G32', 'ro', 'none', '发送池占用块'),
    ('txBufPeak', 1, (2, 22, 0), 'G32', 'ro', 'none', '发送池峰值'),
    ('rxBufUsed', 1, (2, 23, 0), 'G32', 'ro', 'none', '接收池占用块'),
    ('rxBufPeak', 1, (2, 24, 0), 'G32', 'ro', 'none', '接收池峰值'),
    ('rxErrTotal', 1, (2, 25, 0), 'C32', 'ro', 'none', '接收错误合计（相对清零）'),
    ('statsClear', 1, (2, 30, 0), 'INTEGER', 'rw', 'one', '写 1：计数基线清零'),
]
for col, name, acc, chk, note in (
        (2, 'vlId', 'ro', 'none', 'VLID（ROM）'), (3, 'vlBagUs', 'ro', 'none', '活动 BAG(us)'),
        (4, 'vlLmax', 'ro', 'none', '活动 Lmax'), (5, 'vlNetEn', 'ro', 'none', '活动网络使能 bit0=A bit1=B'),
        (6, 'vlShadowBagUs', 'rw', 'bag', '影子 BAG(us)，1000<<k'), (7, 'vlShadowLmax', 'rw', 'lmax', '影子 Lmax 1..1471'),
        (8, 'vlShadowNetEn', 'rw', 'net', '影子网络使能 1..3'), (9, 'vlRomLmax', 'ro', 'none', 'Lmax 硬件上限')):
    MIB.append((name, 1, (3, 1, 1, col), 'G32' if col not in (5, 8) else 'INTEGER', acc, chk, note))
MIB += [
    ('cfgGeneration', 1, (3, 2, 0), 'G32', 'ro', 'none', '活动配置代数'),
    ('cfgCommand', 1, (3, 3, 0), 'INTEGER', 'rw', 'cmd', '1 校验并提交影子 2 回滚 3 影子重载为活动'),
    ('cfgActiveBank', 1, (3, 4, 0), 'INTEGER', 'ro', 'none', '活动 bank'),
    ('cfgLastResult', 1, (3, 5, 0), 'INTEGER', 'ro', 'none', '0 通过 1 BAG 2 Lmax 3 网络 4 带宽 5 回滚 6 自检回滚'),
    ('cfgBudget', 1, (3, 6, 0), 'G32', 'ro', 'none', '128 ms 总线字节预算'),
]
for i, (name, typ, acc, chk, note) in enumerate((
        ('ldState', 'INTEGER', 'ro', 'none', '0 IDLE 1 RECEIVING 2 VERIFYING 3 VERIFIED 4 APPLY_PENDING 5 ACTIVE 6 FAILED 7 ABORTED'),
        ('ldObject', 'INTEGER', 'ro', 'none', '1 配置 2 应用镜像 3 状态 4 故障日志'),
        ('ldBytes', 'G32', 'ro', 'none', '当前会话字节数'), ('ldBlock', 'G32', 'ro', 'none', '当前块号'),
        ('ldCrcExpected', 'G32', 'ro', 'none', '清单 CRC'), ('ldCrcActual', 'G32', 'ro', 'none', '实测 CRC'),
        ('ldLastError', 'INTEGER', 'ro', 'none', '最后错误码'), ('ldFailBlock', 'G32', 'ro', 'none', '失败块号'),
        ('ldCfgVersion', 'G32', 'ro', 'none', '活动配置镜像 generation'),
        ('ldAppBank', 'INTEGER', 'ro', 'none', '应用镜像活动 bank'), ('ldAppLength', 'G32', 'ro', 'none', '应用镜像长度'),
        ('ldAppVersion', 'G32', 'ro', 'none', '应用镜像 generation'), ('ldAbort', 'INTEGER', 'rw', 'one', '写 1 终止当前会话'),
        ('ldLoads', 'C32', 'ro', 'none', '成功加载次数'), ('ldUnloads', 'C32', 'ro', 'none', '卸载次数'),
        ('ldAppCrc', 'G32', 'ro', 'none', '应用镜像 CRC')), 1):
    MIB.append((name, 1, (4, i, 0), typ, acc, chk, note))
for i, (name, typ, acc, chk, note) in enumerate((
        ('rtcTime', 'C64', 'ro', 'none', '64 位 RTC(us)'), ('rtcTimeLow', 'G32', 'ro', 'none', 'RTC 低 32 位'),
        ('rtcState', 'INTEGER', 'ro', 'none', '0 UNSYNCED 1 ACQUIRING 2 LOCKED 3 HOLDOVER 4 DEGRADED 5 FAULT'),
        ('rtcLastOffset', 'INTEGER', 'ro', 'none', '最近偏差 us（饱和）'), ('rtcMaxAbsOffset', 'G32', 'ro', 'none', '最大绝对偏差'),
        ('rtcPathDelay', 'G32', 'ro', 'none', '估算路径时延'), ('rtcFreqCorr', 'INTEGER', 'ro', 'none', '频率校正 2^-32'),
        ('rtcSyncAge', 'G32', 'ro', 'none', '最后同步距今 us'), ('rtcSyncs', 'C32', 'ro', 'none', '成功同步'),
        ('rtcSteps', 'C32', 'ro', 'none', '步进'), ('rtcSlews', 'C32', 'ro', 'none', '微调'),
        ('rtcSourceSwitches', 'C32', 'ro', 'none', '源切换'), ('rtcLost', 'C32', 'ro', 'none', '丢失样本'),
        ('rtcDuplicate', 'C32', 'ro', 'none', '重复'), ('rtcOutOfOrder', 'C32', 'ro', 'none', '乱序/旧序号'),
        ('rtcBadFormat', 'C32', 'ro', 'none', '格式/协议错误'), ('rtcTimeouts', 'C32', 'ro', 'none', '超时'),
        ('rtcRejected', 'C32', 'ro', 'none', '拒绝样本'), ('rtcLastSeq', 'G32', 'ro', 'none', '最后有效序号'),
        ('rtcSourceId', 'G32', 'ro', 'none', '当前时间源'), ('rtcSourceQuality', 'G32', 'ro', 'none', '源质量'),
        ('rtcLastError', 'INTEGER', 'ro', 'none', '最后错误码'),
        ('rtcStepThreshold', 'G32', 'rw', 'nz', '步进门限 us'), ('rtcHoldoverUs', 'G32', 'rw', 'nz', '进入 HOLDOVER 的超时'),
        ('rtcUnsyncUs', 'G32', 'rw', 'nz', '进入 UNSYNCED 的超时'), ('rtcOnewayDelay', 'G32', 'rw', 'any', '单向报文链路时延校正'),
        ('rtcSet', 'OCTET', 'rw', 'oct8', '8 字节大端绝对时间 us（仅写团体字）'),
        ('rtcLastSyncMono', 'G32', 'ro', 'none', '最后同步的单调时间')), 1):
    MIB.append((name, 1, (5, i, 0), typ, acc, chk, note))
for i, (name, typ, acc, chk, note) in enumerate((
        ('trapMask', 'G32', 'rw', 'any', 'bit=1 屏蔽对应事件'), ('trapIntervalMs', 'G32', 'rw', 'nz', '同类事件限速窗口'),
        ('trapDstPort', 'INTEGER', 'rw', 'port', 'Trap 目的端口（默认 162）'),
        ('snmpPort', 'INTEGER', 'rw', 'port', 'SNMP 代理端口（默认 161，响应发出后生效）'),
        ('trapSent', 'C32', 'ro', 'none', '已发送 Trap'), ('trapDropped', 'C32', 'ro', 'none', '合并/屏蔽的事件'),
        ('trapEventType', 'INTEGER', 'ro', 'none', '当前 Trap 事件号'), ('trapEventCount', 'C32', 'ro', 'none', '窗口内累计次数'),
        ('trapEventFirst', 'G32', 'ro', 'none', '首次发生单调时间'), ('trapEventLast', 'G32', 'ro', 'none', '末次发生单调时间'),
        ('mgmtRxAccepted', 'C32', 'ro', 'none', '管理请求接收'), ('mgmtRxDropFull', 'C32', 'ro', 'none', '请求缓冲满丢弃'),
        ('mgmtRxDropPort', 'C32', 'ro', 'none', '未知端口丢弃'), ('snmpInPkts', 'C32', 'ro', 'none', 'SNMP 输入'),
        ('snmpInBadCommunity', 'C32', 'ro', 'none', '团体字错误'), ('snmpInParseErrs', 'C32', 'ro', 'none', 'BER/版本/PDU 错误'),
        ('snmpOutPkts', 'C32', 'ro', 'none', 'SNMP 输出（含 Trap）'), ('errThreshold', 'G32', 'rw', 'nz', '接收错误 Trap 门限'),
        ('linkTimeoutMs', 'G32', 'rw', 'nz', '链路接收超时'), ('rtcPort', 'INTEGER', 'rw', 'port', 'RTC 同步端口'),
        ('tftpRetries', 'C32', 'ro', 'none', 'TFTP 重传'), ('tftpErrors', 'C32', 'ro', 'none', 'TFTP 错误包')), 1):
    MIB.append((name, 1, (6, i, 0), typ, acc, chk, note))
MIB.append(('snmpTrapOID', 2, (0,), 'OID', 'hidden', 'none', '仅用于 Trap'))
TRAPS = ['linkChange', 'rxErrors', 'bufferHigh', 'loaderFault', 'rtcFault', 'cfgChange']


def full_oid(entry):
    return PREFIX[entry[1]] + entry[2]
