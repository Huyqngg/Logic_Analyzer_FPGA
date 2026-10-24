#!/usr/bin/env python3
"""
fpga_link_check.py — chẩn đoán tuyến PC <-> FPGA ĐỘC LẬP với GUI.

Trả lời lần lượt 3 câu hỏi (đúng lớp nào hỏng thì dừng ở lớp đó):
  1. Cổng này có phải USB-UART của FPGA không? (hay là cổng debug của ESP32)
  2. FPGA có sống không? (nạp .sof + reset rồi thì arm trigger CLK 100kHz
     LUÔN CHẠY -> phải thấy 0xA7 TRIGGERED ngay lập tức)
  3. Kênh nào thật sự có tín hiệu tới chân FPGA? (đếm transition từng kênh
     từ 1 lần dump buffer -> lộ ngay dây nào chưa nối / thiếu GND chung)

Chạy:
    python3 fpga_link_check.py                # quét mọi cổng serial
    python3 fpga_link_check.py /dev/ttyUSB2   # kiểm tra đúng 1 cổng
    python3 fpga_link_check.py /dev/ttyUSB2 --trig 1   # trigger CH-1 thay vì CH-16
"""

import sys
import time

import serial
import serial.tools.list_ports

BAUD = 115200

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

SCALER  = 1      # 50 MHz, 1 tick = 20 ns
PRECAP  = 1024


def hexdump(data, limit=32):
    s = " ".join(f"{b:02X}" for b in data[:limit])
    return s + (" ..." if len(data) > limit else "")


def looks_like_text(data) -> bool:
    """Debug console ESP32 = ASCII in được; stream FPGA = binary A1/A3/..."""
    if len(data) < 8:
        return False
    printable = sum(1 for b in data if 32 <= b < 127 or b in (10, 13, 9))
    return printable / len(data) > 0.8


def drain(ser, seconds):
    out = bytearray()
    t0 = time.time()
    while time.time() - t0 < seconds:
        chunk = ser.read(256)
        if chunk:
            out.extend(chunk)
    return bytes(out)


def wait_for_byte(ser, wanted, timeout):
    """Đọc tới khi gặp byte `wanted`. Trả (found, mọi_byte_đã_thấy)."""
    seen = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout:
        chunk = ser.read(64)
        if not chunk:
            continue
        seen.extend(chunk)
        if wanted in chunk:
            return True, bytes(seen)
    return False, bytes(seen)


def send_cmd(ser, cmd, val):
    ser.write(bytes([cmd, val]))


def check_port(port, trig_ch=15, trig_rising=True):
    print(f"\n{'='*64}\n>>> KIỂM TRA CỔNG {port}  (trigger CH-{trig_ch+1}, "
          f"{'Rising' if trig_rising else 'Falling'})\n{'='*64}")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.2)
    except Exception as e:
        print(f"  [X] Không mở được cổng: {e}")
        return False

    try:
        # ---- BƯỚC 1: nghe im lặng 2s — cổng ESP32 sẽ tự phát text debug ----
        ser.reset_input_buffer()
        noise = drain(ser, 2.0)
        if noise:
            print(f"  [i] Cổng TỰ PHÁT {len(noise)} byte khi chưa hỏi gì: {hexdump(noise)}")
            if looks_like_text(noise):
                try:
                    print(f"      Nội dung: {noise[:80].decode(errors='replace')!r}")
                except Exception:
                    pass
                print("  [X] Đây gần như chắc chắn là CỔNG DEBUG CỦA ESP32 (in text định kỳ).")
                print("      -> GUI phải chọn cổng USB-UART nối PIN_99/PIN_100 của FPGA, KHÔNG phải cổng này.")
                return False
        else:
            print("  [i] Cổng im lặng khi chưa hỏi (đúng kiểu FPGA đứng chờ lệnh).")

        # ---- BƯỚC 2: reset FSM + gửi config, arm trigger ----
        send_cmd(ser, CMD_SET_STOP, 0x01); time.sleep(0.05)
        send_cmd(ser, CMD_SET_STOP, 0x00)
        send_cmd(ser, CMD_SET_SCALER, (SCALER >> 8) & 0xFF)
        send_cmd(ser, CMD_SET_SCALER, SCALER & 0xFF)
        send_cmd(ser, CMD_SET_PRECAP_DEPTH, (PRECAP >> 8) & 0xFF)
        send_cmd(ser, CMD_SET_PRECAP_DEPTH, PRECAP & 0xFF)
        send_cmd(ser, CMD_SET_CHANNEL, trig_ch)
        send_cmd(ser, CMD_SET_TRIG_TYPE, 1 if trig_rising else 0)
        send_cmd(ser, CMD_SET_DELAY_EN, 0x00)
        send_cmd(ser, CMD_SET_DELAY_VAL, 0x00)
        send_cmd(ser, CMD_SET_START_READ, 0x00)
        ser.reset_input_buffer()
        send_cmd(ser, CMD_SET_ENABLE, 0x01)
        print(f"  [i] Đã gửi config (scaler={SCALER} -> 50MHz, precap={PRECAP}) + ENABLE. Chờ 0xA7...")

        ok_trig, seen = wait_for_byte(ser, HDR_TRIGGERED, timeout=3.0)
        if not ok_trig:
            send_cmd(ser, CMD_SET_ENABLE, 0x00)
            if seen:
                print(f"  [X] Có {len(seen)} byte về nhưng KHÔNG có 0xA7: {hexdump(seen)}")
                print("      -> Thiết bị trả lời nhưng không đúng giao thức LA (cổng khác? baud khác?).")
            else:
                print("  [X] FPGA IM LẶNG HOÀN TOÀN sau khi arm. Nguyên nhân thường gặp:")
                print("      1. FPGA chưa được nạp .sof (Cyclone IV MẤT cấu hình khi tắt nguồn -> nạp lại")
                print("         quartus_test/output_files/logic_analyzer_top.sof mỗi lần cấp điện).")
                print("      2. Chưa nhấn reset i_rstn (PIN_88) sau khi nạp.")
                print("      3. Đấu chéo TX/RX sai: o_tx PIN_100 -> RX của USB-UART, i_rx PIN_99 <- TX.")
                print("      4. USB-UART không phải mức 3.3V, hoặc thiếu GND chung FPGA <-> USB-UART.")
                if trig_ch == 15:
                    print("      (Trigger đang đặt ở CH-16 = CLK 100kHz luôn chạy; nếu FPGA sống mà")
                    print("       kênh này không có tín hiệu thì vẫn không nổ -> thử --trig khác sau khi")
                    print("       đã loại trừ 1-4.)")
            return False
        print("  [OK] Nhận 0xA7 TRIGGERED -> FPGA sống + nhận lệnh + kênh trigger có cạnh!")

        # ---- BƯỚC 3: chờ DONE rồi dump buffer ----
        ok_done, _ = wait_for_byte(ser, HDR_DONE, timeout=3.0)
        if not ok_done:
            print("  [!] Không thấy 0xAF DONE sau 3s (buffer không đầy?) — vẫn thử đọc.")
        send_cmd(ser, CMD_SET_START_READ, 0x01)
        raw = bytearray()
        t0 = time.time()
        while time.time() - t0 < 8.0:          # 8192 record x 4B @115200 ~ 2.9s
            chunk = ser.read(4096)
            if chunk:
                raw.extend(chunk)
            elif raw:
                break                           # đã có dữ liệu và dòng đã cạn
        send_cmd(ser, CMD_SET_START_READ, 0x00)
        send_cmd(ser, CMD_SET_ENABLE, 0x00)

        # ---- BƯỚC 4: đếm transition từng kênh ----
        records = []
        i = 0
        while i + 3 < len(raw):
            if raw[i] in (HDR_PRE_BUFF, HDR_POST_BUFF):
                ts, lo, hi = raw[i+1], raw[i+2], raw[i+3]
                records.append((ts, (hi << 8) | lo))
                i += 4
            else:
                i += 1
        if not records:
            print(f"  [X] Đọc được {len(raw)} byte nhưng không parse ra record nào: {hexdump(raw)}")
            return False

        total_ticks = sum(r[0] for r in records)
        print(f"  [OK] Dump {len(records)} record, tổng {total_ticks} tick "
              f"= {total_ticks * (SCALER + 1) * 10 / 1e6:.2f} ms cửa sổ.")

        trans = [0] * 16
        prev = records[0][1]
        for _ts, word in records[1:]:
            diff = word ^ prev
            for c in range(16):
                if diff & (1 << c):
                    trans[c] += 1
            prev = word
        last = records[-1][1]

        print("  ---- Tín hiệu tới chân FPGA (transition / mức hiện tại) ----")
        expected = ["UART-A TX", "UART-A RX", "I2C-A SDA", "I2C-A SCL",
                    "SPI SCK", "SPI MOSI", "SPI MISO", "SPI CS",
                    "UART-B", "I2C-B SDA", "I2C-B SCL", "PWM 1kHz",
                    "PWM 10kHz", "CLK 1MHz", "CLK 500kHz", "CLK 100kHz"]
        n_alive = 0
        for c in range(16):
            alive = trans[c] > 0
            n_alive += alive
            mark = "SỐNG" if alive else "chết"
            print(f"    CH-{c+1:2d} ({expected[c]:10s}): {trans[c]:5d} transition, "
                  f"mức={(last >> c) & 1}   [{mark}]")
        if n_alive == 0:
            print("  [X] Không kênh nào có tín hiệu: ESP32 chưa chạy / chưa nối dây / THIẾU GND CHUNG.")
        elif trans[13] == 0 and trans[14] == 0 and trans[15] == 0:
            print("  [!] 3 kênh CLK (luôn chạy) đều chết -> nghi sai thứ tự dây i_probe hoặc ESP32 chưa chạy.")
        else:
            print(f"  [OK] {n_alive}/16 kênh sống. Tuyến PC<->FPGA<->ESP32 hoạt động.")
            print("       -> Vào GUI: Trigger CH-1, Falling, Precapture 1024, Divisor 1, Auto Run + Auto Decode.")
        return True
    finally:
        try:
            ser.close()
        except Exception:
            pass


def main():
    args = [a for a in sys.argv[1:]]
    trig_ch = 15                       # CH-16 = CLK 100kHz: chắc chắn có cạnh
    if "--trig" in args:
        k = args.index("--trig")
        trig_ch = int(args[k + 1]) - 1
        del args[k:k + 2]

    ports = args or [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        print("Không tìm thấy cổng serial nào. Cắm USB-UART rồi chạy lại.")
        return

    print(f"Sẽ kiểm tra: {', '.join(ports)}")
    good = [p for p in ports if check_port(p, trig_ch=trig_ch)]
    print(f"\n{'='*64}")
    if good:
        print(f"KẾT LUẬN: cổng FPGA hoạt động = {', '.join(good)} -> chọn cổng này trong GUI.")
    else:
        print("KẾT LUẬN: chưa cổng nào hoạt động đúng — xem chẩn đoán từng cổng ở trên.")


if __name__ == "__main__":
    main()
