"""
test_decode_offline.py  (P3)  — bản v2 "IC thật"
================================================
Test OFFLINE (không cần phần cứng) cho predict + decode, mô phỏng đúng cách FPGA
ghi dữ liệu (x_axis = HỢP mọi transition).

Bao phủ:
  1. Sơ đồ 16 kênh v2: 2 UART, 2 I2C (đa-bus), 1 SPI 4-dây (MOSI+MISO), PWM, CLK.
  2. SPI đọc CẢ 2 chiều (MOSI + MISO).
  3. I2C real: multi-byte + ACK/NACK + repeated-START + clock-stretching.
  4. UART có parity (chẵn) + 2 stop-bit.
  5. CHỐNG false-positive: PWM/clock KHÔNG bị nhận là I2C_SDA.
  6. PERMUTATION-INVARIANT: xáo trộn 16 kênh -> predict/decode vẫn đúng.
  7. HARDWARE-REALISM: chèn rollover timestamp 8-bit như FPGA thật.

Chạy:  cd gui && python test_decode_offline.py
"""

from __future__ import annotations

import random
from bisect import bisect_right

from sample_access import SampleAccessor
from channel_predictor import ChannelRole, RoleAccumulator, predict_roles
from decoders import (decode_all, decode_uart, decode_spi, decode_i2c,
                      estimate_uart_bit_ticks)

# ---- cấu hình mô phỏng (miền tick @ 50MHz: scaler_val=2) --------------------
SCALER_VAL = 2
SR = 1e9 / (SCALER_VAL * 10.0)          # 50 MHz
T0, TEND = 0, 240000                     # cửa sổ ~4.8ms


# ===========================================================================
#  Bộ phát tín hiệu: mỗi kênh là danh sách keyframe (tick, level)
# ===========================================================================
class Gen:
    def __init__(self, init_level=0, t0=T0, tend=TEND):
        self.t0 = t0
        self.tend = tend
        self.kf = [(t0, init_level)]

    def emit(self, tick, level):
        if tick <= self.kf[-1][0]:
            tick = self.kf[-1][0] + 1
        if level != self.kf[-1][1]:
            self.kf.append((int(tick), level))

    def level_at(self, t):
        ticks = [k[0] for k in self.kf]
        i = bisect_right(ticks, t) - 1
        return self.kf[max(0, i)][1]

    def keyframe_ticks(self):
        return [k[0] for k in self.kf if self.t0 <= k[0] <= self.tend]


def gen_clock(period, duty=0.5, start=T0, tend=TEND):
    g = Gen(0, tend=tend)
    t = start
    while t < tend:
        g.emit(t, 1)
        g.emit(t + period * duty, 0)
        t += period
    return g


def gen_uart(byte_list, bit_ticks, start=500, reps=8, slot=28000,
             parity="N", stop_bits=1):
    """UART idle-high, LSB-first, có parity (N/E/O) + stop-bit."""
    g = Gen(1)
    for r in range(reps):
        t = start + r * slot
        for byte in byte_list:
            g.emit(t, 0); t += bit_ticks                  # start
            ones = 0
            for n in range(8):
                b = (byte >> n) & 1
                ones += b
                g.emit(t, b); t += bit_ticks
            if parity in ("E", "O"):
                pb = (ones % 2) if parity == "E" else (1 - ones % 2)
                g.emit(t, pb); t += bit_ticks
            for _ in range(stop_bits):
                g.emit(t, 1); t += bit_ticks              # stop
            t += bit_ticks * 3                             # idle giữa byte
    return g


# ---- SPI 4 dây: master gửi MOSI, slave trả MISO --------------------------
def _spi_burst4(cs, sck, mosi, miso, mosi_bytes, miso_bytes, t, sck_period):
    for bi, mbyte in enumerate(mosi_bytes):
        sbyte = miso_bytes[bi % len(miso_bytes)]
        cs.emit(t, 0)
        tb = t + sck_period
        for i in range(8):                  # MSB first
            base = tb + i * sck_period
            mosi.emit(base, (mbyte >> (7 - i)) & 1)   # master out
            miso.emit(base, (sbyte >> (7 - i)) & 1)   # slave out
            sck.emit(base + sck_period * 0.25, 1)     # cạnh lên -> sample
            sck.emit(base + sck_period * 0.75, 0)
        cs.emit(tb + 8 * sck_period + sck_period, 1)
        t = tb + 8 * sck_period + sck_period * 13
    return t


def gen_spi(mosi_bytes, miso_bytes, sck_period=50, start=800, reps=16, slot=13000):
    cs, sck, mosi, miso = Gen(1), Gen(0), Gen(0), Gen(0)
    for r in range(reps):
        _spi_burst4(cs, sck, mosi, miso, mosi_bytes, miso_bytes,
                    start + r * slot, sck_period)
    return cs, sck, mosi, miso


# ---- I2C real: START/byte/ACK/repeated-START/STOP + clock-stretch --------
def _i2c_start(scl, sda, t, bit):
    q = bit // 4
    scl.emit(t, 1); sda.emit(t, 1)
    sda.emit(t + q, 0)                      # SDA↓ khi SCL cao
    return t + bit


def _i2c_repeated_start(scl, sda, t, bit):
    q = bit // 4
    scl.emit(t, 0); sda.emit(t + q, 1)      # nhả SDA cao lúc SCL thấp
    scl.emit(t + 2 * q, 1)                   # SCL lên
    sda.emit(t + 3 * q, 0)                   # SDA↓ khi SCL cao -> repeated START
    return t + bit


def _i2c_byte(scl, sda, val, t, bit, stretch_bit=-1, stretch=0):
    q = bit // 4
    for k in range(8):
        extra = stretch if k == stretch_bit else 0   # clock-stretch: kéo dài SCL thấp
        scl.emit(t, 0)
        sda.emit(t + q + extra, (val >> (7 - k)) & 1)
        scl.emit(t + 2 * q + extra, 1)               # cạnh lên -> sample
        t += bit + extra
    return t


def _i2c_ack(scl, sda, ack, t, bit):
    q = bit // 4
    scl.emit(t, 0); sda.emit(t + q, 0 if ack == 0 else 1)
    scl.emit(t + 2 * q, 1)
    return t + bit


def _i2c_stop(scl, sda, t, bit):
    q = bit // 4
    scl.emit(t, 0); sda.emit(t + q, 0)
    scl.emit(t + 2 * q, 1)                   # SCL cao
    sda.emit(t + 3 * q, 1)                   # SDA↑ -> STOP
    return t + 2 * bit


def _i2c_write_txn(scl, sda, addr, datas, t, bit, ack=0):
    t = _i2c_start(scl, sda, t, bit)
    t = _i2c_byte(scl, sda, (addr << 1) | 0, t, bit)
    t = _i2c_ack(scl, sda, ack, t, bit)
    for d in datas:
        t = _i2c_byte(scl, sda, d, t, bit)
        t = _i2c_ack(scl, sda, ack, t, bit)
    t = _i2c_stop(scl, sda, t, bit)
    return t


def gen_i2c(addr, datas, bit=125, start=1000, reps=8, slot=28000, ack=0):
    scl, sda = Gen(1), Gen(1)
    for r in range(reps):
        _i2c_write_txn(scl, sda, addr, datas, start + r * slot, bit, ack)
    return scl, sda


# ===========================================================================
#  Dựng waveform kiểu-FPGA từ 16 Gen
# ===========================================================================
def build_samples(gens_by_channel, t0=T0, tend=TEND):
    all_ticks = {t0, tend}
    for g in gens_by_channel:
        all_ticks.update(g.keyframe_ticks())
    x_axis = sorted(all_ticks)
    channel_data = [[g.level_at(t) for t in x_axis] for g in gens_by_channel]
    return SampleAccessor(x_axis, channel_data, SCALER_VAL, num_channels=16)


def inject_rollover(sa, max_gap=255):
    xs = list(sa.x_axis)
    new = []
    for i in range(len(xs) - 1):
        new.append(xs[i])
        t = xs[i] + max_gap
        while t < xs[i + 1]:
            new.append(t)
            t += max_gap
    new.append(xs[-1])
    cd = [[sa.level_at(c, t) for t in new] for c in range(16)]
    return SampleAccessor(new, cd, sa.scaler_val, num_channels=16)


# ===========================================================================
#  Kịch bản ground-truth (khớp sơ đồ 16 kênh v2)
# ===========================================================================
UART_A = [0x55, 0xAA, 0x00, 0xFF, 0x3C]
UART_B = [0x48, 0x69, 0x21]              # "Hi!"
SPI_MOSI_BYTES = [0xAA, 0x55, 0x3C]
SPI_MISO_BYTES = [0x1F, 0x2E, 0x3D]
I2C_A = (0x3C, [0xAA, 0x55])
I2C_B = (0x50, [0x12])


# ===========================================================================
#  REALISTIC REGIME — mô phỏng ĐÚNG capture phần cứng thật:
#  buffer RLE 8192 record bị 5 kênh CLK/PWM liên tục lấp đầy sau ~133k tick
#  (~2.66ms @50MHz) < 1 vòng loop ESP32 (~3.6ms) => mỗi cửa sổ chứa TỐI ĐA
#  1 burst mỗi giao thức (có thể cụt / vắng mặt). Theo esp32_gen_input.ino v2.
# ===========================================================================
REAL_WINDOW = 133000
UART_BIT_A = SR / 115200.0               # ~434 tick (HW UART)
UART_BIT_B = 450                          # bit-bang 9us -> ~111111 baud (lệch 3.7%)
I2C_BIT_A = 500                           # 100kHz
I2C_BIT_B = 340                           # bit-bang ~6.8us/bit
SPI_PERIOD = 50                           # 1MHz
SPI_MOSI_REAL = [0xAA, 0x55, 0x3C, 0x00]
SPI_MISO_REAL = [0x1F, 0x2E, 0x3D, 0x4C]
ALL_BURSTS = {"uart_a", "uart_a_rx", "uart_b", "i2c_a", "i2c_b", "spi"}


def _uart_burst(g, byte_list, bit_ticks, start):
    """1 burst UART: các byte nối đuôi nhau (HW UART flush liên tục)."""
    t = start
    for byte in byte_list:
        g.emit(t, 0); t += bit_ticks                  # start
        for n in range(8):
            g.emit(t, (byte >> n) & 1); t += bit_ticks
        g.emit(t, 1); t += bit_ticks                  # stop
    return int(t)


def _spi_frame_real(cs, sck, mosi, miso, mosi_bytes, miso_bytes, t, sck_period):
    """1 khung CS duy nhất chứa TOÀN BỘ các byte (ESP32 giữ CS thấp suốt)."""
    cs.emit(t, 0)
    tb = t + 2 * sck_period
    for bi, mbyte in enumerate(mosi_bytes):
        sbyte = miso_bytes[bi % len(miso_bytes)]
        for i in range(8):                            # MSB first, mode 0
            base = tb + i * sck_period
            mosi.emit(base, (mbyte >> (7 - i)) & 1)
            miso.emit(base, (sbyte >> (7 - i)) & 1)
            sck.emit(base + sck_period * 0.25, 1)     # cạnh lên -> sample
            sck.emit(base + sck_period * 0.75, 0)
        tb += 8 * sck_period + 2 * sck_period         # nghỉ nhỏ giữa byte
    cs.emit(tb + sck_period, 1)
    return int(tb + 2 * sck_period)


def _i2c_txn_real(scl, sda, t, ack=0):
    """Giao dịch I2C-A thật: write 0xAA 0x55 -> repeated-START -> read 1 byte+NACK."""
    bit = I2C_BIT_A
    t = _i2c_start(scl, sda, t, bit)
    t = _i2c_byte(scl, sda, (0x3C << 1) | 0, t, bit)
    t = _i2c_ack(scl, sda, ack, t, bit)
    for d in (0xAA, 0x55):
        t = _i2c_byte(scl, sda, d, t, bit)
        t = _i2c_ack(scl, sda, ack, t, bit)
    t = _i2c_repeated_start(scl, sda, t, bit)
    t = _i2c_byte(scl, sda, (0x3C << 1) | 1, t, bit)
    t = _i2c_ack(scl, sda, ack, t, bit)
    t = _i2c_byte(scl, sda, 0xA5, t, bit)
    t = _i2c_ack(scl, sda, 1, t, bit)                 # master NACK kết thúc đọc
    t = _i2c_stop(scl, sda, t, bit)
    return int(t)


def gen_realistic_window(burst_offset=20000, include=None, miso_jumper=True,
                         window=REAL_WINDOW):
    """
    Dựng 16 kênh của 1 cửa sổ capture thật (sơ đồ .ino v2):
      CH0/1 UART-A TX/RX, CH2/3 I2C-A SDA/SCL, CH4-7 SPI SCK/MOSI/MISO/CS,
      CH8 UART-B (bit=450), CH9/10 I2C-B (NACK-only), CH11-15 PWM/CLK liên tục.
    Lịch burst TUẦN TỰ như loop() ESP32, bắt đầu tại burst_offset (sát biên ->
    tự nhiên bị cắt cụt). `include`: giao thức nào có mặt trong cửa sổ này.
    Trả (gens, expected) — expected chỉ chứa kênh có vai trò chắc chắn.
    """
    include = set(include) if include is not None else set(ALL_BURSTS)

    def mk(level):
        return Gen(level, tend=window)

    uart_a, uart_a_rx = mk(1), mk(1)
    sda_a, scl_a = mk(1), mk(1)
    sck, mosi, miso, cs = mk(0), mk(0), mk(0), mk(1)
    uart_b = mk(1)
    sda_b, scl_b = mk(1), mk(1)

    t = burst_offset
    if "uart_a" in include:
        _uart_burst(uart_a, UART_A, UART_BIT_A, t)
    if "uart_a_rx" in include:
        _uart_burst(uart_a_rx, UART_B, UART_BIT_A, t)      # "Hi!" song song TX
    if "uart_a" in include or "uart_a_rx" in include:
        t += int(len(UART_A) * 10 * UART_BIT_A) + 800
    if "uart_b" in include:
        t = _uart_burst(uart_b, UART_A, UART_BIT_B, t) + 800
    if "i2c_a" in include:
        t = _i2c_txn_real(scl_a, sda_a, t) + 800
    if "i2c_b" in include:
        t = _i2c_write_txn(scl_b, sda_b, I2C_B[0], I2C_B[1], t, I2C_BIT_B, ack=1) + 800
    if "spi" in include:
        _spi_frame_real(cs, sck, mosi, miso if miso_jumper else mk(0),
                        SPI_MOSI_REAL, SPI_MISO_REAL, t, SPI_PERIOD)

    gens = [uart_a, uart_a_rx, sda_a, scl_a, sck, mosi, miso, cs, uart_b,
            sda_b, scl_b,
            gen_clock(int(SR / 1000), duty=0.75, tend=window),      # PWM 1kHz 75%
            gen_clock(int(SR / 10000), duty=0.25, tend=window),     # PWM 10kHz 25%
            gen_clock(int(SR / 1000000), duty=0.5, tend=window),    # CLK 1MHz
            gen_clock(int(SR / 500000), duty=0.5, tend=window),     # CLK 500kHz
            gen_clock(int(SR / 100000), duty=0.5, tend=window)]     # CLK 100kHz

    expected = {11: "PWM", 12: "PWM", 13: "CLOCK", 14: "CLOCK", 15: "CLOCK"}
    if "uart_a" in include:
        expected[0] = "UART"
    if "uart_a_rx" in include:
        expected[1] = "UART"
    if "i2c_a" in include:
        expected[2] = "I2C_SDA"; expected[3] = "I2C_SCL"
    if "spi" in include:
        expected[4] = "SPI_SCK"; expected[5] = "SPI_DATA"; expected[7] = "SPI_CS"
        if miso_jumper:
            expected[6] = "SPI_DATA"
    if "uart_b" in include:
        expected[8] = "UART"
    if "i2c_b" in include:
        expected[9] = "I2C_SDA"; expected[10] = "I2C_SCL"
    return gens, expected


def build_scenario():
    uart_bit = SR / 115200.0
    cs, sck, mosi, miso = gen_spi(SPI_MOSI_BYTES, SPI_MISO_BYTES)
    scl_a, sda_a = gen_i2c(I2C_A[0], I2C_A[1], start=1000)
    scl_b, sda_b = gen_i2c(I2C_B[0], I2C_B[1], start=14000)   # lệch pha bus A
    gens = {
        0:  gen_uart(UART_A, uart_bit),                 # UART-A TX
        1:  gen_uart(UART_B, uart_bit, start=4000),     # UART-A RX
        2:  sda_a, 3:  scl_a,                            # I2C-A
        4:  sck, 5:  mosi, 6:  miso, 7:  cs,             # SPI-A (4 dây)
        8:  gen_uart(UART_A, uart_bit, start=8000),     # UART-B TX
        9:  sda_b, 10: scl_b,                            # I2C-B
        11: gen_clock(int(SR / 5000), duty=0.75),       # PWM 5kHz 75%
        12: gen_clock(int(SR / 10000), duty=0.25),      # PWM 10kHz 25%
        13: gen_clock(int(SR / 1000000), duty=0.5),     # CLK 1MHz
        14: gen_clock(int(SR / 500000), duty=0.5),      # CLK 500kHz
        15: gen_clock(int(SR / 100000), duty=0.5),      # CLK 100kHz
    }
    expected = {
        0: "UART", 1: "UART", 2: "I2C_SDA", 3: "I2C_SCL",
        4: "SPI_SCK", 5: "SPI_DATA", 6: "SPI_DATA", 7: "SPI_CS",
        8: "UART", 9: "I2C_SDA", 10: "I2C_SCL",
        11: "PWM", 12: "PWM", 13: "CLOCK", 14: "CLOCK", 15: "CLOCK",
    }
    return [gens[i] for i in range(16)], expected


# ===========================================================================
#  Kiểm tra
# ===========================================================================
def _norm(role):
    """MOSI/MISO không phân biệt được theo hướng -> gộp thành SPI_DATA khi so."""
    return "SPI_DATA" if role in ("SPI_MOSI", "SPI_MISO") else role


def _hex_bytes(anns, ch):
    return [int(a.text[2:4], 16) for a in anns
            if a.ch == ch and a.kind == "byte" and a.text.startswith("0x")]


def check_roles(roles, expected, tag=""):
    ok = 0
    for r in roles:
        exp = expected.get(r.ch)
        if _norm(r.role) == exp:
            ok += 1
        else:
            print(f"   >> CH{r.ch}: got {r.role:12s} exp {exp}  ({r.label}, conf={r.confidence:.2f})")
    print(f"  [{tag}] role accuracy = {ok}/{len(expected)}")
    return ok


def _cycles_ok(got, pattern):
    if len(got) < len(pattern):
        return False
    return all(got[i] == pattern[i % len(pattern)] for i in range(len(got)))


def run_once(gens, expected, tag, assert_decode=True):
    sa = build_samples(gens)
    print(f"\n=== {tag} === events={sa.n}  window={sa.duration_ticks} ticks")
    roles = predict_roles(sa, uart_baud=115200)
    ok = check_roles(roles, expected, tag)
    anns = decode_all(sa, roles, uart_baud=115200)

    if assert_decode:
        # UART: mỗi kênh UART phải ra chuỗi byte lặp đúng 1 trong 2 pattern.
        for r in [x for x in roles if x.role == "UART"]:
            ub = _hex_bytes(anns, r.ch)
            assert _cycles_ok(ub, UART_A) or _cycles_ok(ub, UART_B), \
                f"UART decode sai [{tag}] CH{r.ch}: {[hex(x) for x in ub[:6]]}"
        # SPI: 2 kênh data phải ra {MOSI pattern, MISO pattern} (đổi chỗ được).
        data_ch = [x.ch for x in roles if x.role in ("SPI_MOSI", "SPI_MISO")]
        if len(data_ch) == 2:
            b0 = _hex_bytes(anns, data_ch[0])
            b1 = _hex_bytes(anns, data_ch[1])
            good = ((_cycles_ok(b0, SPI_MOSI_BYTES) and _cycles_ok(b1, SPI_MISO_BYTES)) or
                    (_cycles_ok(b1, SPI_MOSI_BYTES) and _cycles_ok(b0, SPI_MISO_BYTES)))
            assert good, f"SPI MOSI/MISO decode sai [{tag}]: {[hex(x) for x in b0[:4]]} / {[hex(x) for x in b1[:4]]}"
        # I2C: có START + đúng >=1 địa chỉ mong đợi.
        i2c_addr_texts = [a.text for a in anns if a.kind == "byte" and a.text.startswith("A:")]
        assert any("0x3C" in t for t in i2c_addr_texts), f"I2C-A addr 0x3C không thấy [{tag}]"
    return sa, roles, anns, ok


def permutation_test(base_gens, base_expected, seed):
    perm = list(range(16))
    random.Random(seed).shuffle(perm)
    new_gens = [base_gens[perm[i]] for i in range(16)]
    new_expected = {i: base_expected[perm[i]] for i in range(16)}
    return run_once(new_gens, new_expected, f"PERMUTATION seed={seed}")


# ---- các test tập trung ---------------------------------------------------
def test_false_positive_pwm_not_i2c():
    """PWM + 1 gated-clock: PWM KHÔNG được nhận là I2C_SDA."""
    scl, sda = gen_i2c(0x3C, [0xAA])
    gens = [Gen(1) for _ in range(16)]
    gens[0] = sda; gens[1] = scl                        # 1 bus I2C thật
    gens[2] = gen_clock(int(SR / 5000), duty=0.75)      # PWM (bẫy false-positive)
    gens[3] = gen_clock(int(SR / 1000000), duty=0.5)    # CLK
    sa = build_samples(gens)
    roles = predict_roles(sa, uart_baud=115200)
    r2 = next(r for r in roles if r.ch == 2)
    assert r2.role in ("PWM", "CLOCK"), f"PWM bị nhận nhầm thành {r2.role}!"
    assert {roles[0].role, roles[1].role} == {"I2C_SDA", "I2C_SCL"}, \
        f"I2C thật không được nhận: {roles[0].role}/{roles[1].role}"
    print("  [false-positive] PWM -> PWM/CLOCK (không phải I2C_SDA)  OK")


def test_i2c_repeated_start_and_stretch():
    """Write-then-read (repeated-START) + clock-stretch -> decode đúng addr/R-W."""
    scl, sda = Gen(1), Gen(1)
    bit = 125
    t = 1000
    for _ in range(4):
        t = _i2c_start(scl, sda, t, bit)
        t = _i2c_byte(scl, sda, (0x3C << 1) | 0, t, bit, stretch_bit=3, stretch=600)  # write + stretch
        t = _i2c_ack(scl, sda, 0, t, bit)
        t = _i2c_byte(scl, sda, 0x10, t, bit)
        t = _i2c_ack(scl, sda, 0, t, bit)
        t = _i2c_repeated_start(scl, sda, t, bit)
        t = _i2c_byte(scl, sda, (0x3C << 1) | 1, t, bit)   # read
        t = _i2c_ack(scl, sda, 0, t, bit)
        t = _i2c_byte(scl, sda, 0x55, t, bit)
        t = _i2c_ack(scl, sda, 1, t, bit)                  # NACK kết thúc đọc
        t = _i2c_stop(scl, sda, t, bit)
        t += 4000
    anns = decode_i2c(build_samples([sda, scl] + [Gen(0)] * 14), 1, 0)
    addrs = [a.text for a in anns if a.kind == "byte" and a.text.startswith("A:")]
    assert any(a == "A:0x3CW" for a in addrs), f"Không thấy write 0x3C: {addrs[:6]}"
    assert any(a == "A:0x3CR" for a in addrs), f"Không thấy read 0x3C (repeated-START): {addrs[:6]}"
    assert any(a.kind == "nack" for a in anns), "Không thấy NACK"
    print(f"  [i2c-advanced] repeated-START + clock-stretch OK ({len(addrs)} addr)")


def test_uart_parity():
    """UART parity chẵn + 2 stop-bit -> decode đúng."""
    bit = SR / 115200.0
    g = gen_uart([0x41, 0x42, 0x43], bit, parity="E", stop_bits=2)   # "ABC"
    sa = build_samples([g] + [Gen(0)] * 15)
    anns = decode_uart(sa, 0, baud=115200, parity="E", stop_bits=2)
    vals = [int(a.text[2:4], 16) for a in anns if a.kind == "byte"]
    assert _cycles_ok(vals, [0x41, 0x42, 0x43]), f"UART parity decode sai: {[hex(v) for v in vals[:6]]}"
    print(f"  [uart-parity] even parity + 2 stop OK ({[hex(v) for v in vals[:3]]})")


# ---- các test REALISTIC (capture 1-burst như phần cứng thật) ---------------
def _predict_real(gens):
    sa = build_samples(gens, tend=REAL_WINDOW)
    return sa, predict_roles(sa, uart_baud=115200)


def test_realistic_full_window():
    """Cửa sổ realistic đầy đủ (+3 hoán vị): đúng role, đúng byte decode,
    SDA không bao giờ thành UART, kênh burst không bao giờ thành CLOCK/PWM."""
    for seed in (None, 1, 2, 3):
        gens, expected = gen_realistic_window()
        perm = list(range(16))
        tag = "REALISTIC"
        if seed is not None:
            random.Random(seed).shuffle(perm)
            gens = [gens[perm[i]] for i in range(16)]
            expected = {i: expected[perm[i]] for i in range(16)}
            tag = f"REALISTIC perm={seed}"
        inv = {perm[i]: i for i in range(16)}     # kênh gốc -> vị trí mới

        sa, roles = _predict_real(gens)
        ok = check_roles(roles, expected, tag)
        assert ok == 16, f"[{tag}] role accuracy {ok}/16"
        for r in roles:                            # không bao giờ SAI-MẠNH
            exp = expected.get(r.ch)
            if exp == "I2C_SDA":
                assert r.role != "UART", f"[{tag}] SDA CH{r.ch} bị gán UART!"
            if exp not in ("PWM", "CLOCK"):
                assert r.role not in ("PWM", "CLOCK"), \
                    f"[{tag}] kênh burst CH{r.ch} thành {r.role}!"

        anns = decode_all(sa, roles, uart_baud=115200)
        assert _hex_bytes(anns, inv[0]) == UART_A, \
            f"[{tag}] UART-A decode: {[hex(v) for v in _hex_bytes(anns, inv[0])]}"
        assert _hex_bytes(anns, inv[1]) == UART_B, \
            f"[{tag}] UART-RX decode: {[hex(v) for v in _hex_bytes(anns, inv[1])]}"
        assert _hex_bytes(anns, inv[8]) == UART_A, \
            f"[{tag}] UART-B(111kbd) decode: {[hex(v) for v in _hex_bytes(anns, inv[8])]}"
        texts_a = [a.text for a in anns if a.ch == inv[2]]
        assert "A:0x3CW" in texts_a and "A:0x3CR" in texts_a, \
            f"[{tag}] I2C-A thiếu addr W/R (repeated-START): {texts_a[:8]}"
        texts_b = [a.text for a in anns if a.ch == inv[9]]
        assert "A:0x50W" in texts_b and "0x12" in texts_b, \
            f"[{tag}] I2C-B decode sai: {texts_b[:8]}"
        b0 = _hex_bytes(anns, inv[5]); b1 = _hex_bytes(anns, inv[6])
        good = ((b0 == SPI_MOSI_REAL and b1 == SPI_MISO_REAL) or
                (b1 == SPI_MOSI_REAL and b0 == SPI_MISO_REAL))
        assert good, f"[{tag}] SPI decode sai: {[hex(v) for v in b0]} / {[hex(v) for v in b1]}"
    print("  [realistic-full] 4/4 cửa sổ (gốc + 3 hoán vị): 16/16 role + byte đúng")


def test_uart_autobaud_111111():
    """UART-B bit-bang ~111111 baud (lệch 3.7%): auto-baud phải giữ bit đo được
    (~450 tick), KHÔNG snap về 434 (115200) -> decode đúng đủ 5 byte."""
    gens, _ = gen_realistic_window(include={"uart_b"})
    sa = build_samples(gens, tend=REAL_WINDOW)
    bit = estimate_uart_bit_ticks(sa, 8, baud=115200)
    assert 440 <= bit <= 460, f"auto-baud bit={bit:.1f}, phải ~450 (không snap 434)"
    vals = _hex_bytes(decode_uart(sa, 8, baud=115200), 8)
    assert vals == UART_A, f"UART-B decode sai: {[hex(v) for v in vals]}"
    roles = predict_roles(sa, uart_baud=115200)
    assert roles[8].role == "UART", f"UART-B không được nhận: {roles[8].role}"
    print(f"  [uart-111kbd] bit đo được = {bit:.0f} tick, decode 5/5 byte OK")


def test_i2c_nack_only():
    """I2C-B không có slave (NACK 100%): vẫn phải nhận cặp + decode addr/data."""
    gens, _ = gen_realistic_window(include={"i2c_b"})
    sa, roles = _predict_real(gens)
    assert roles[9].role == "I2C_SDA" and roles[10].role == "I2C_SCL", \
        f"I2C-B NACK-only không được nhận: CH9={roles[9].role} CH10={roles[10].role}"
    anns = decode_i2c(sa, 10, 9)
    texts = [a.text for a in anns]
    assert "A:0x50W" in texts and "0x12" in texts, f"I2C-B decode sai: {texts[:8]}"
    assert any(a.kind == "nack" for a in anns), "Thiếu NACK"
    assert not any(a.kind == "ack" for a in anns), "Không được có ACK (bus không slave)"
    print("  [i2c-nack-only] cặp nhận đúng + decode A:0x50W 0x12 NACK OK")


def test_truncated_decodes():
    """Giao dịch bị cửa sổ cắt cụt: decode phần có, KHÔNG sinh byte rác."""
    # a. I2C mất STOP (cửa sổ hết giữa giao dịch)
    gens, _ = gen_realistic_window(include={"i2c_a"}, burst_offset=REAL_WINDOW - 12000)
    sa = build_samples(gens, tend=REAL_WINDOW)
    anns = decode_i2c(sa, 3, 2)
    texts = [a.text for a in anns if a.kind == "byte"]
    assert texts and texts[0] == "A:0x3CW", f"I2C cụt đuôi: {texts[:4]}"
    assert not any(a.kind == "stop" for a in anns), "Không được có STOP (đã bị cắt)"

    # b. UART đứt giữa byte thứ 3 -> chỉ ra 2 byte ĐẦY ĐỦ, không byte rác
    gens, _ = gen_realistic_window(include={"uart_a"},
                                   burst_offset=REAL_WINDOW - int(2.5 * 10 * UART_BIT_A))
    sa = build_samples(gens, tend=REAL_WINDOW)
    vals = _hex_bytes(decode_uart(sa, 0, baud=115200), 0)
    assert vals == UART_A[:2], f"UART cụt đuôi phải ra đúng 2 byte đầu: {[hex(v) for v in vals]}"

    # c. SPI: CS không kịp nhả (cửa sổ hết khi CS còn thấp) -> decode byte đã clock
    gens, _ = gen_realistic_window(include={"spi"}, burst_offset=REAL_WINDOW - 1100)
    sa = build_samples(gens, tend=REAL_WINDOW)
    vals = _hex_bytes(decode_spi(sa, 4, 5, 7), 5)
    assert vals and vals == SPI_MOSI_REAL[:len(vals)], \
        f"SPI cụt đuôi decode sai: {[hex(v) for v in vals]}"

    # d. SPI CỤT ĐẦU: CS đã thấp ngay từ t0 (không có cạnh xuống trong cửa sổ)
    cs, sck = Gen(0, tend=REAL_WINDOW), Gen(0, tend=REAL_WINDOW)
    mosi, miso = Gen(0, tend=REAL_WINDOW), Gen(0, tend=REAL_WINDOW)
    _spi_frame_real(cs, sck, mosi, miso, SPI_MOSI_REAL, SPI_MISO_REAL, 100, SPI_PERIOD)
    sa = build_samples([cs, sck, mosi, miso] + [Gen(1, tend=REAL_WINDOW)] * 12,
                       tend=REAL_WINDOW)
    vals = _hex_bytes(decode_spi(sa, 1, 2, 0), 2)
    assert vals == SPI_MOSI_REAL, f"SPI cụt đầu decode sai: {[hex(v) for v in vals]}"
    print("  [truncated] I2C mất STOP / UART đứt giữa byte / SPI cụt đầu+đuôi: 0 byte rác")


def test_cross_capture_merge():
    """RoleAccumulator: 2 cửa sổ thiếu bù nhau -> đủ 16 role; cửa sổ rỗng không
    hạ cấp; gán tay (set_manual) luôn thắng."""
    acc = RoleAccumulator(16)

    # Cửa sổ A: KHÔNG có SPI (burst trượt ra ngoài)
    gens, _ = gen_realistic_window(include=ALL_BURSTS - {"spi"})
    _, roles_a = _predict_real(gens)
    d = {r.ch: r.role for r in acc.observe(roles_a)}
    assert d[0] == "UART" and d[2] == "I2C_SDA", "cửa sổ A phải nhận UART/I2C"
    assert d[4] in ("IDLE", "UNKNOWN"), f"SPI chưa xuất hiện mà CH4={d[4]}"

    # Cửa sổ B: CÓ SPI nhưng thiếu I2C-A + UART-A -> merge phải ĐỦ hết
    gens, _ = gen_realistic_window(include={"spi", "uart_b", "i2c_b"})
    _, roles_b = _predict_real(gens)
    d = {r.ch: r.role for r in acc.observe(roles_b)}
    assert d[0] == "UART" and d[1] == "UART" and d[8] == "UART", "mất UART từ cửa sổ A!"
    assert d[2] == "I2C_SDA" and d[3] == "I2C_SCL", "mất I2C-A từ cửa sổ A!"
    assert d[9] == "I2C_SDA" and d[10] == "I2C_SCL", "mất I2C-B!"
    assert (d[4], d[7]) == ("SPI_SCK", "SPI_CS") and \
        d[5] in ("SPI_MOSI", "SPI_MISO") and d[6] in ("SPI_MOSI", "SPI_MISO"), \
        f"SPI từ cửa sổ B không vào merge: {d[4]}/{d[5]}/{d[6]}/{d[7]}"
    assert all(d[c] in ("PWM", "CLOCK") for c in (11, 12, 13, 14, 15))

    # Cửa sổ C: RỖNG (chỉ clock liên tục) -> không được hạ cấp gì
    gens, _ = gen_realistic_window(include=set())
    _, roles_c = _predict_real(gens)
    d2 = {r.ch: r.role for r in acc.observe(roles_c)}
    assert d2 == d, f"cửa sổ rỗng đã hạ cấp niềm tin: {d2}"

    # set_manual khóa kênh: các observe sau không ghi đè
    acc.set_manual([ChannelRole(0, "SPI_CS", "spi-cs", 9, 1.0)])
    gens, _ = gen_realistic_window()
    _, roles_full = _predict_real(gens)
    d3 = {r.ch: r.role for r in acc.observe(roles_full)}
    assert d3[0] == "SPI_CS", f"manual lock bị ghi đè: CH0={d3[0]}"
    print("  [merge] A(thiếu SPI)+B(thiếu I2C-A) -> đủ 16; rỗng không hạ cấp; manual thắng")


def test_single_burst_gating():
    """Capture 1-burst: kênh giao thức GATED; 5 kênh liên tục NOT gated."""
    gens, _ = gen_realistic_window(burst_offset=30000)
    sa = build_samples(gens, tend=REAL_WINDOW)
    for ch, name in [(0, "UART-A"), (2, "SDA-A"), (3, "SCL-A"), (7, "SPI-CS")]:
        assert sa.is_gated(ch), f"{name} (CH{ch}) phải GATED (1 burst + idle biên)"
    for ch in (12, 13, 14, 15):
        assert not sa.is_gated(ch), f"CH{ch} liên tục không được coi là gated"
    assert sa.activity_coverage(3) < 0.4, \
        f"SCL burst coverage={sa.activity_coverage(3):.2f} phải < 0.4"
    assert sa.activity_coverage(13) > 0.9, \
        f"CLK 1MHz coverage={sa.activity_coverage(13):.2f} phải > 0.9"
    d = sa.duty_active(11)
    assert 0.70 <= d <= 0.80, f"duty_active PWM 1kHz = {d:.2f}, phải ~0.75"
    print("  [realistic-gating] 1-burst gated / liên tục not-gated / coverage OK")


def test_two_spi_frames_cs_not_pwm():
    """Cửa sổ dài hơn 1 vòng loop ESP32 -> chứa 2 khung CS. Bug thật đã gặp:
    2 xung CS + nghỉ dài ở giữa bị nhận nhầm 'pwm-333Hz-99%' -> CS bị nuốt
    -> SCK/MOSI rơi xuống unknown. CS phải là SPI_CS, SPI nhận đủ bộ."""
    W = 194000
    def mk(level):
        return Gen(level, tend=W)
    sck, mosi, miso, cs = mk(0), mk(0), mk(0), mk(1)
    for t0 in (14000, 176000):            # 2 giao dịch cách ~160k tick (2 vòng loop)
        _spi_frame_real(cs, sck, mosi, miso, SPI_MOSI_REAL, SPI_MISO_REAL,
                        t0, SPI_PERIOD)
    gens = [mk(1), mk(1), mk(1), mk(1), sck, mosi, miso, cs,
            mk(1), mk(1), mk(1),
            gen_clock(int(SR / 1000), duty=0.75, tend=W),
            gen_clock(int(SR / 10000), duty=0.25, tend=W),
            gen_clock(int(SR / 1000000), duty=0.5, tend=W),
            gen_clock(int(SR / 500000), duty=0.5, tend=W),
            gen_clock(int(SR / 100000), duty=0.5, tend=W)]
    sa = build_samples(gens, tend=W)
    roles = predict_roles(sa, uart_baud=115200)
    assert roles[7].role not in ("PWM", "CLOCK"), \
        f"CS 2-khung bị nhận nhầm {roles[7].role} ({roles[7].label})!"
    assert roles[7].role == "SPI_CS", f"CS phải là SPI_CS, ra {roles[7].role}"
    assert roles[4].role == "SPI_SCK", f"SCK ra {roles[4].role}"
    assert roles[5].role in ("SPI_MOSI", "SPI_MISO"), f"MOSI ra {roles[5].role}"
    assert roles[6].role in ("SPI_MOSI", "SPI_MISO"), f"MISO ra {roles[6].role}"
    print("  [2-khung-CS] CS=SPI_CS (không thành PWM), SCK/MOSI/MISO đủ bộ  OK")


def main():
    gens, expected = build_scenario()
    sa, roles, anns, ok = run_once(gens, expected, "BASELINE (sơ đồ v2)")
    assert ok >= 15, f"predict baseline chỉ đúng {ok}/16"

    total_ok, n_runs = ok, 1
    for seed in range(8):
        _, _, _, ok2 = permutation_test(gens, expected, seed)
        assert ok2 >= 15, f"predict permutation seed={seed} chỉ đúng {ok2}/16"
        total_ok += ok2
        n_runs += 1

    print("\n--- HARDWARE REALISM (chèn rollover timestamp 8-bit) ---")
    sa_hw = inject_rollover(build_samples(gens), max_gap=13)
    print(f"events: thường={sa.n} -> có rollover={sa_hw.n}")
    roles_hw = predict_roles(sa_hw, uart_baud=115200)
    ok_hw = check_roles(roles_hw, expected, "HW-rollover")
    assert ok_hw >= 15, f"predict với rollover chỉ đúng {ok_hw}/16"

    print("\n--- TEST TẬP TRUNG ---")
    test_false_positive_pwm_not_i2c()
    test_i2c_repeated_start_and_stretch()
    test_uart_parity()

    print("\n--- REALISTIC REGIME (capture 1-burst như phần cứng) ---")
    test_single_burst_gating()
    test_two_spi_frames_cs_not_pwm()
    test_realistic_full_window()
    test_uart_autobaud_111111()
    test_i2c_nack_only()
    test_truncated_decodes()
    test_cross_capture_merge()

    print("\n" + "=" * 62)
    print(f"TẤT CẢ PASS. Trung bình role accuracy = {total_ok}/{16*n_runs} "
          f"= {100*total_ok/(16*n_runs):.1f}%")
    print("Permutation-invariant + hardware-realism + IC-thật (MISO/repeated-START/")
    print("clock-stretch/parity) + chống false-positive: ĐỀU ĐÚNG.")
    print("REALISTIC: cửa sổ 133k tick 1-burst (cụt/vắng/NACK/111kbd) + merge")
    print("đa-capture (RoleAccumulator): ĐỀU ĐÚNG.")
    print("=" * 62)


if __name__ == "__main__":
    main()
