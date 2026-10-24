#!/usr/bin/env python3
"""
capture_dump.py — chụp 1 cửa sổ capture THẬT từ FPGA và lưu ra JSON,
để phân tích / regression offline mà không cần cắm lại phần cứng.

Chạy:
    python3 capture_dump.py --port /dev/ttyUSB2 --trig 4 --edge fall \
        --scaler 1 --precap 4096 --out dumps/trig4_fall_s1.json

File JSON: {"scaler_val": s+1, "trig_ch": n, "edge": "...",
            "records": [[ts, word16], ...]}
"""

import argparse
import json
import os
import time

import serial

CMD_SET_SCALER       = 0xFA
CMD_SET_PRECAP_DEPTH = 0xFE
CMD_SET_CHANNEL      = 0xFB
CMD_SET_TRIG_TYPE    = 0xFC
CMD_SET_DELAY_EN     = 0xF7
CMD_SET_DELAY_VAL    = 0xF8
CMD_SET_ENABLE       = 0xFD
CMD_SET_STOP         = 0xFF
CMD_SET_START_READ   = 0xF9

HDR_PRE_BUFF  = 0xA1
HDR_POST_BUFF = 0xA3
HDR_TRIGGERED = 0xA7
HDR_DONE      = 0xAF


def wait_for(ser, wanted, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        chunk = ser.read(64)
        if chunk and wanted in chunk:
            return True
    return False


def capture_once(port, trig_ch0, rising, scaler, precap, timeout=6.0):
    """Trả list [(ts, word16)] của 1 capture, hoặc None nếu không trigger."""
    with serial.Serial(port, 115200, timeout=0.2) as ser:
        ser.write(bytes([CMD_SET_STOP, 0x01])); time.sleep(0.05)
        ser.write(bytes([CMD_SET_STOP, 0x00]))
        ser.write(bytes([CMD_SET_SCALER, (scaler >> 8) & 0xFF]))
        ser.write(bytes([CMD_SET_SCALER, scaler & 0xFF]))
        ser.write(bytes([CMD_SET_PRECAP_DEPTH, (precap >> 8) & 0xFF]))
        ser.write(bytes([CMD_SET_PRECAP_DEPTH, precap & 0xFF]))
        ser.write(bytes([CMD_SET_CHANNEL, trig_ch0]))
        ser.write(bytes([CMD_SET_TRIG_TYPE, 1 if rising else 0]))
        ser.write(bytes([CMD_SET_DELAY_EN, 0x00]))
        ser.write(bytes([CMD_SET_DELAY_VAL, 0x00]))
        ser.write(bytes([CMD_SET_START_READ, 0x00]))
        ser.reset_input_buffer()
        ser.write(bytes([CMD_SET_ENABLE, 0x01]))

        if not wait_for(ser, HDR_TRIGGERED, timeout):
            ser.write(bytes([CMD_SET_ENABLE, 0x00]))
            return None
        wait_for(ser, HDR_DONE, timeout)

        ser.write(bytes([CMD_SET_START_READ, 0x01]))
        raw = bytearray()
        t0 = time.time()
        while time.time() - t0 < 10.0:
            chunk = ser.read(4096)
            if chunk:
                raw.extend(chunk)
            elif raw:
                break
        ser.write(bytes([CMD_SET_START_READ, 0x00]))
        ser.write(bytes([CMD_SET_ENABLE, 0x00]))

    records = []
    i = 0
    while i + 3 < len(raw):
        if raw[i] in (HDR_PRE_BUFF, HDR_POST_BUFF):
            records.append((raw[i + 1], (raw[i + 3] << 8) | raw[i + 2]))
            i += 4
        else:
            i += 1
    return records


def load_dump(path):
    """Đọc JSON dump -> (x_axis, channel_data, scaler_val) như GUI dựng."""
    with open(path) as f:
        d = json.load(f)
    x_axis, channel_data = [], [[] for _ in range(16)]
    total = 0
    for ts, word in d["records"]:
        total += ts
        x_axis.append(total)
        for c in range(16):
            channel_data[c].append((word >> c) & 1)
    return x_axis, channel_data, d["scaler_val"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB2")
    ap.add_argument("--trig", type=int, default=1, help="kênh trigger 1-16")
    ap.add_argument("--edge", choices=["rise", "fall"], default="fall")
    ap.add_argument("--scaler", type=int, default=1, help="0=100MHz, 1=50MHz...")
    ap.add_argument("--precap", type=int, default=4096)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    recs = capture_once(args.port, args.trig - 1, args.edge == "rise",
                        args.scaler, args.precap)
    if not recs:
        print(f"[X] Không trigger được (CH-{args.trig} {args.edge}).")
        return 1
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w") as f:
        json.dump({"scaler_val": args.scaler + 1, "trig_ch": args.trig,
                   "edge": args.edge, "precap": args.precap,
                   "records": recs}, f)
    ticks = sum(r[0] for r in recs)
    print(f"[OK] {args.out}: {len(recs)} record, {ticks} tick "
          f"({ticks * (args.scaler + 1) * 10 / 1e6:.2f} ms)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
