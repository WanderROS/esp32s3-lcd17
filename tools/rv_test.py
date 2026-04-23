#!/usr/bin/env python3
"""
RAMViewer 串口手动测试工具
用法: python3 rv_test.py [port] [baud]
默认: port=/dev/tty.usbserial-* baud=115200
"""

import sys
import time
import struct
import serial
import serial.tools.list_ports

# ── CRC16-CCITT ──────────────────────────────────────────────────────────────
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
        crc &= 0xFFFF
    return crc

# ── 帧构造 ────────────────────────────────────────────────────────────────────
SOF = 0xAA
CMD_READ_VAR  = 0x01
CMD_WRITE_VAR = 0x02
CMD_READ_RESP = 0x81
CMD_WRITE_RESP= 0x82
CMD_ERROR     = 0xFF
CMD_PING      = 0x10
CMD_PONG      = 0x90
NO_BITFIELD   = 0xFF

def build_frame(cmd: int, seq: int, payload: bytes = b'') -> bytes:
    hdr = bytes([SOF, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF, cmd, seq])
    crc = crc16(hdr + payload)
    return hdr + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def build_ping(seq=1) -> bytes:
    return build_frame(CMD_PING, seq)

def build_read(addr: int, size: int, seq=1) -> bytes:
    """读取 addr 处 size 字节（非 bitfield）"""
    entry = struct.pack('<IHBB', addr, size, NO_BITFIELD, 0x00)
    payload = bytes([1]) + entry   # count=1
    return build_frame(CMD_READ_VAR, seq, payload)

def build_read_multi(vars_: list, seq=1) -> bytes:
    """vars_: [(addr, size), ...]"""
    payload = bytes([len(vars_)])
    for addr, size in vars_:
        payload += struct.pack('<IHBB', addr, size, NO_BITFIELD, 0x00)
    return build_frame(CMD_READ_VAR, seq, payload)

# ── 帧解析 ────────────────────────────────────────────────────────────────────
CMD_NAMES = {
    CMD_READ_RESP:  'READ_RESP',
    CMD_WRITE_RESP: 'WRITE_RESP',
    CMD_ERROR:      'ERROR',
    CMD_PONG:       'PONG',
}
ERR_NAMES = {0x00:'OK', 0x01:'CRC', 0x02:'ADDR', 0x03:'SIZE', 0x04:'CMD'}

def recv_frame(ser: serial.Serial, timeout=2.0) -> dict | None:
    """阻塞等待一帧，返回解析结果或 None"""
    buf = bytearray()
    deadline = time.time() + timeout
    state = 'IDLE'
    payload_len = 0

    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        byte = b[0]

        if state == 'IDLE':
            if byte == SOF:
                buf = bytearray([byte])
                state = 'LEN_L'
        elif state == 'LEN_L':
            buf.append(byte); payload_len = byte; state = 'LEN_H'
        elif state == 'LEN_H':
            buf.append(byte); payload_len |= byte << 8; state = 'CMD'
        elif state == 'CMD':
            buf.append(byte); state = 'SEQ'
        elif state == 'SEQ':
            buf.append(byte)
            state = 'PAYLOAD' if payload_len > 0 else 'CRC_L'
        elif state == 'PAYLOAD':
            buf.append(byte)
            if len(buf) == 5 + payload_len:
                state = 'CRC_L'
        elif state == 'CRC_L':
            buf.append(byte); state = 'CRC_H'
        elif state == 'CRC_H':
            buf.append(byte)
            # 验证 CRC
            recv_crc = buf[-2] | (buf[-1] << 8)
            calc_crc = crc16(bytes(buf[:-2]))
            if recv_crc != calc_crc:
                print(f'  [!] CRC 错误: recv={recv_crc:04X} calc={calc_crc:04X}')
                return None
            cmd = buf[3]; seq = buf[4]
            payload = bytes(buf[5:5+payload_len])
            return {'cmd': cmd, 'seq': seq, 'payload': payload, 'raw': bytes(buf)}

    return None  # timeout

def print_response(resp: dict | None):
    if resp is None:
        print('  ✗ 超时，无响应')
        return
    cmd = resp['cmd']
    name = CMD_NAMES.get(cmd, f'0x{cmd:02X}')
    print(f'  ← {name} seq={resp["seq"]} payload={resp["payload"].hex(" ").upper() or "(空)"}')
    if cmd == CMD_PONG:
        print('  ✓ PONG 收到，MCU 通信正常！')
    elif cmd == CMD_READ_RESP:
        p = resp['payload']
        count = p[0] if p else 0
        print(f'  ✓ 读取成功，count={count}，数据: {p[1:].hex(" ").upper()}')
        # 尝试按 uint32 解析
        data = p[1:]
        for i in range(0, len(data)-3, 4):
            val = struct.unpack_from('<I', data, i)[0]
            print(f'     [{i//4}] 0x{val:08X} = {val}')
    elif cmd == CMD_ERROR:
        ec = resp['payload'][0] if resp['payload'] else 0xFF
        print(f'  ✗ 错误: {ERR_NAMES.get(ec, hex(ec))}')

# ── 主程序 ────────────────────────────────────────────────────────────────────
def find_port():
    ports = list(serial.tools.list_ports.comports())
    # 优先 USB CDC（usbmodem），其次 USB-TTL（usbserial）
    for p in ports:
        if 'usbmodem' in p.device.lower():
            return p.device
    for p in ports:
        if 'usbserial' in p.device.lower() or 'cu.usb' in p.device.lower():
            return p.device
    if ports:
        return ports[0].device
    return None

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    if not port:
        print('找不到串口，请手动指定: python3 rv_test.py /dev/tty.usbserial-XXXX')
        sys.exit(1)

    print(f'连接 {port} @ {baud}')
    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    # ── 先监听 1 秒，看 MCU 有没有主动输出（日志/乱码）──────────────────────
    print('\n[0] 监听 1 秒原始输出...')
    deadline = time.time() + 1.0
    raw = bytearray()
    while time.time() < deadline:
        b = ser.read(64)
        if b:
            raw.extend(b)
    if raw:
        print(f'  收到 {len(raw)} 字节: {raw[:64].hex(" ").upper()}')
        try:
            print(f'  文本: {raw.decode("utf-8", errors="replace")[:120]}')
        except Exception:
            pass
    else:
        print('  (无数据)')

    # ── 测试 1: PING ──────────────────────────────────────────────────────────
    print('\n[1] PING 测试')
    ser.reset_input_buffer()
    frame = build_ping(seq=1)
    print(f'  → {frame.hex(" ").upper()}')
    ser.write(frame)
    ser.flush()
    print_response(recv_frame(ser, timeout=2.0))

    # ── 测试 2: 读 ESP32-S3 内部 DRAM（0x3FC88000，voice_state 附近）──────────
    print('\n[2] 读 DRAM 0x3FC88000 (4 bytes)')
    ser.reset_input_buffer()
    frame = build_read(0x3FC88000, 4, seq=2)
    print(f'  → {frame.hex(" ").upper()}')
    ser.write(frame)
    ser.flush()
    print_response(recv_frame(ser, timeout=2.0))

    # ── 测试 3: 读 ESP32-S3 PSRAM 起始（0x3C000000，4 bytes）────────────────
    print('\n[3] 读 PSRAM 0x3C000000 (4 bytes)')
    ser.reset_input_buffer()
    frame = build_read(0x3C000000, 4, seq=3)
    print(f'  → {frame.hex(" ").upper()}')
    ser.write(frame)
    ser.flush()
    print_response(recv_frame(ser, timeout=2.0))

    # ── 测试 4: 故意发 CRC 错误帧（期望收到 ERROR 响应）────────────────────
    print('\n[4] 故意发 CRC 错误帧（期望 ERROR 响应）')
    ser.reset_input_buffer()
    bad = build_ping(seq=4)
    bad = bad[:-1] + bytes([bad[-1] ^ 0xFF])  # 破坏最后一字节
    print(f'  → {bad.hex(" ").upper()}')
    ser.write(bad)
    ser.flush()
    print_response(recv_frame(ser, timeout=2.0))

    ser.close()
    print('\n完成')

if __name__ == '__main__':
    main()
