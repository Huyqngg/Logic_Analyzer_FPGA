"""
decoders.py  (P3)
=================
Decode các giao thức từ waveform đã capture, làm việc HOÀN TOÀN trong miền tick
qua `SampleAccessor.level_at`  =>  ĐỘC LẬP VỊ TRÍ DÂY (permutation-invariant).

Đầu vào: SampleAccessor + danh sách ChannelRole (vai trò từng kênh).
Đầu ra : List[Annotation]  (byte + marker) để GUI overlay lên sóng.

Bản v2 "real-grade" (chạy đúng với IC THẬT của thầy):
    - SPI : 4 dây (SCK/MOSI/MISO/CS), decode CẢ 2 chiều, đủ 4 mode, MSB/LSB.
    - UART: cấu hình data-bit (5-9), parity (N/E/O), stop-bit (1/2); auto-baud GCD.
    - I2C : repeated-START + clock-stretching (bám cạnh SCL THẬT) + ACK/NACK + R/W.

Không import tkinter/matplotlib -> unit-test offline được.

Vai trò (role) dùng chung với channel_predictor:
    CLOCK, PWM,
    SPI_SCK, SPI_MOSI, SPI_MISO, SPI_CS,
    I2C_SCL, I2C_SDA,
    UART,
    COUNTER_BIT,
    IDLE, UNKNOWN
"""

from __future__ import annotations

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from math import gcd
from typing import Dict, List, Optional, Tuple

from sample_access import SampleAccessor


# ----------------------------------------------------------------- data model
@dataclass
class Annotation:
    ch: int          # kênh để vẽ overlay
    t_start: int     # tick bắt đầu
    t_end: int       # tick kết thúc (== t_start nếu là marker vạch)
    text: str        # nội dung hiển thị ('0x55', "'U'", 'START', '1MHz'...)
    kind: str        # 'byte' | 'start' | 'stop' | 'ack' | 'nack' | 'clock' | 'counter' | 'frame'


# --------------------------------------------------------------- role helpers
def _by_role(roles, role: str) -> List[int]:
    return [r.ch for r in roles if r.role == role]


def _groups(roles) -> Dict[Optional[int], List]:
    """Gom role theo group_id (dùng cho SPI/I2C nhiều bus)."""
    out: Dict[Optional[int], List] = {}
    for r in roles:
        out.setdefault(r.group_id, []).append(r)
    return out


# ===========================================================================
#  UART  (cấu hình data-bit / parity / stop-bit -> chạy với IC thật)
# ===========================================================================
STANDARD_BAUDS = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]


def measure_uart_bit_ticks(sa: SampleAccessor, ch: int) -> float:
    """
    ĐO số tick / 1 bit UART từ chính waveform (không cần biết baud).

    KHÔNG dùng GCD: phần cứng thật (ESP32 HW-UART) phát cạnh bám lưới ~1us
    -> run xen kẽ 400/450 tick, GCD sập về 50 rồi rơi về min = SAI 8%.
    Thay bằng 2 bước bền vững:
      1. Cụm run 1-bit = các run <= 1.35x min -> bit thô = median cụm.
      2. Tinh chỉnh: run w gồm k = round(w/bit_thô) bit -> bit = tổng w / tổng k
         (bình quân trọng số, triệt lượng tử hóa + jitter).
    Trả 0.0 nếu không đo được.
    """
    ws = [w for w in sa.run_widths(ch, trim_boundary=True) if w >= 4]  # bỏ glitch
    if len(ws) < 4:
        return 0.0
    w_min = min(ws)
    ws = [w for w in ws if w <= 10 * w_min]      # bỏ idle giữa các burst
    ones = sorted(w for w in ws if w <= 1.35 * w_min)
    bit0 = float(ones[len(ones) // 2])
    total_w = total_k = 0
    for w in ws:
        k = max(1, round(w / bit0))
        total_w += w
        total_k += k
    return (total_w / total_k) if total_k else bit0


def estimate_uart_bit_ticks(sa: SampleAccessor, ch: int, baud: Optional[int] = None) -> float:
    """
    Số tick / 1 bit UART. `baud` là GỢI Ý chứ không phải ép buộc:
      1. Đo thật bằng measure_uart_bit_ticks.
      2. Nếu số đo khớp baud gợi ý trong 3% -> snap về nominal (sạch hơn số đo).
      3. Nếu không, snap về baud chuẩn gần nhất trong 2.5%.
      4. Nếu không nữa -> giữ số đo (vd UART bit-bang ~111111 baud lệch 3.7%
         so với 115200 -> decode bằng bit ĐO ĐƯỢC mới chính xác).
    """
    measured = measure_uart_bit_ticks(sa, ch)
    if measured <= 0:
        return sa.sample_rate_hz / baud if baud else 0.0
    if baud:
        nominal = sa.sample_rate_hz / baud
        if abs(measured - nominal) / nominal <= 0.03:
            return nominal
    best = None
    for b in STANDARD_BAUDS:
        nb = sa.sample_rate_hz / b
        rel = abs(measured - nb) / nb
        if rel <= 0.025 and (best is None or rel < best[0]):
            best = (rel, nb)
    return best[1] if best else measured


def _parity_ok(value: int, data_bits: int, bit_val: int, parity: str) -> bool:
    ones = bin(value & ((1 << data_bits) - 1)).count("1")
    if parity in ("E", "e"):          # even: tổng số 1 (data+parity) chẵn
        return ((ones + bit_val) % 2) == 0
    if parity in ("O", "o"):          # odd
        return ((ones + bit_val) % 2) == 1
    return True


def decode_uart(sa: SampleAccessor, ch: int, baud: Optional[int] = None,
                data_bits: int = 8, parity: str = "N", stop_bits: int = 1,
                msb_first: bool = False) -> List[Annotation]:
    """
    UART idle-high. Cạnh xuống = start bit. Sample giữa mỗi bit.
    Hỗ trợ data_bits 5-9, parity N/E/O, stop 1/2. Trả 1 'byte' cho mỗi frame hợp lệ.

    Chống nhiễu / chống nhầm (quan trọng với capture thật):
      - KIỂM TRA ỔN ĐỊNH BIT-CELL: mọi cạnh của kênh bên trong khung ứng viên
        phải nằm sát ranh giới bit (±0.25 bit). Cạnh giữa ô bit => không phải
        UART (vd I2C SDA, khung bị cửa sổ cắt giữa byte) -> bỏ khung im lặng.
      - Resync đúng cuối khung (N bit) để KHÔNG nhảy qua start-bit của byte kế
        khi các byte nối đuôi nhau (HW UART gửi back-to-back).
    """
    bit = estimate_uart_bit_ticks(sa, ch, baud)
    out: List[Annotation] = []
    if bit <= 0:
        return out

    parity_slots = 0 if parity in ("N", "n") else 1
    n_slots = 1 + data_bits + parity_slots + stop_bits     # tổng số bit của khung
    falls = sa.edges(ch, "falling")
    all_edges = sa.edges(ch, "both")
    i = 0
    while i < len(falls):
        t0 = falls[i]
        # start bit phải = 0 tại tâm
        if sa.level_at(ch, t0 + 0.5 * bit) != 0:
            i += 1
            continue
        value = 0
        ok = True
        for n in range(data_bits):
            center = t0 + (1.5 + n) * bit
            if center > sa.t_end:
                ok = False
                break
            b = sa.level_at(ch, center)
            if msb_first:
                value = (value << 1) | b
            else:
                value |= (b << n)
        if not ok:
            break

        # parity bit (nếu có)
        if parity_slots:
            pc = t0 + (1.5 + data_bits) * bit
            if pc > sa.t_end:
                break
            pbit = sa.level_at(ch, pc)
            if not _parity_ok(value, data_bits, pbit, parity):
                i += 1
                continue  # sai parity -> khung không hợp lệ

        # stop bit(s) phải = 1
        stop0 = t0 + (1.5 + data_bits + parity_slots) * bit
        stop_ok = True
        for s in range(stop_bits):
            sc = stop0 + s * bit
            if sc <= sa.t_end and sa.level_at(ch, sc) != 1:
                stop_ok = False
                break
        if not stop_ok:
            i += 1
            continue

        # Ổn định bit-cell: cạnh trong khung phải sát ranh giới k*bit (±0.35 bit).
        # 0.35 đủ rộng cho jitter bit-bang thật (ngắt ESP32 xê dịch vài us) mà vẫn
        # giết cạnh GIỮA ô bit (0.5 bit) của I2C SDA / cửa sổ cắt ngang byte.
        frame_end_f = t0 + n_slots * bit
        lo = bisect_right(all_edges, t0)
        hi = bisect_left(all_edges, min(frame_end_f, sa.t_end))
        stable = True
        for te in all_edges[lo:hi]:
            k = round((te - t0) / bit)
            if abs(te - (t0 + k * bit)) > 0.35 * bit:
                stable = False
                break
        if not stable:
            i += 1
            continue

        t_frame_end = int(frame_end_f)
        printable = chr(value) if 32 <= value < 127 else ""
        text = f"0x{value:02X}" + (f" '{printable}'" if printable else "")
        out.append(Annotation(ch, int(t0), t_frame_end, text, "byte"))

        # Nhảy qua khung này; chừa lại start-bit kế (có thể rơi ĐÚNG cuối khung
        # khi byte nối đuôi nhau) -> mốc lùi 0.25 bit.
        resync = t0 + (n_slots - 0.25) * bit
        i += 1
        while i < len(falls) and falls[i] < resync:
            i += 1
    return out


def uart_frame_quality(sa: SampleAccessor, ch: int, baud: Optional[int] = None,
                       data_bits: int = 8, parity: str = "N", stop_bits: int = 1
                       ) -> Tuple[List[Annotation], float, float]:
    """
    Đánh giá chất lượng giả thuyết 'kênh này là UART' cho predictor:
      (frames, edge_explanation, bit_ticks)
    edge_explanation = tỉ lệ cạnh CỦA CHÍNH KÊNH nằm trong các khung decode được.
    UART thật: gần như mọi cạnh thuộc về khung (~1.0). Kênh giả (SDA, nhiễu):
    vài khung tình cờ chỉ giải thích được phần nhỏ số cạnh -> loại.
    """
    frames = decode_uart(sa, ch, baud=baud, data_bits=data_bits,
                         parity=parity, stop_bits=stop_bits)
    bit = estimate_uart_bit_ticks(sa, ch, baud)
    edges = sa.edges(ch, "both")
    if not edges:
        return frames, 0.0, bit
    spans = sorted((f.t_start, f.t_end) for f in frames if f.kind == "byte")
    covered = 0
    si = 0
    for t in edges:
        while si < len(spans) and spans[si][1] < t:
            si += 1
        if si < len(spans) and spans[si][0] <= t <= spans[si][1]:
            covered += 1
    return frames, covered / len(edges), bit


# ===========================================================================
#  SPI  (SCK + MOSI + CS [+ MISO], đủ 4 mode, MSB-first mặc định)
# ===========================================================================
def decode_spi(sa: SampleAccessor, sck: int, mosi: int, cs: int,
               miso: Optional[int] = None,
               cpol: int = 0, cpha: int = 0, msb_first: bool = True,
               bits_per_word: int = 8, cs_active_low: bool = True) -> List[Annotation]:
    """
    Trong mỗi khung CS-active, tại cạnh 'sample' của SCK đọc MOSI (và MISO nếu có)
    -> gom thành word. Mode0/3 (cpol==cpha): lấy mẫu cạnh LÊN; Mode1/2: cạnh XUỐNG.
    Byte MOSI (master->slave) đặt trên kênh MOSI; byte MISO (slave->master) trên MISO.
    """
    out: List[Annotation] = []
    sample_rising = (cpol == cpha)
    sck_kind = "rising" if sample_rising else "falling"

    # Khung CS: cặp (cạnh vào-active, cạnh ra-active).
    enter = sa.edges(cs, "falling" if cs_active_low else "rising")
    leave = sa.edges(cs, "rising" if cs_active_low else "falling")
    frames: List[tuple] = []
    # Khung CỤT ĐẦU: CS đã active ngay đầu cửa sổ (capture cắt giữa giao dịch)
    # -> decode từ t0 tới cạnh ra-active đầu tiên.
    active_lv = 0 if cs_active_low else 1
    if sa.level_at(cs, sa.t0) == active_lv and leave:
        if not enter or leave[0] < enter[0]:
            frames.append((sa.t0, leave[0]))
    for tf in enter:
        tr = next((t for t in leave if t > tf), sa.t_end)
        frames.append((tf, tr))
    if not frames:
        if sa.level_at(cs, sa.t0) == active_lv:   # CS kẹt active suốt -> 1 khung
            frames = [(sa.t0, sa.t_end)]
        else:
            return out    # CS nghỉ suốt cửa sổ (block không có giao dịch SPI)

    for (t_lo, t_hi) in frames:
        clk_edges = [t for t in sa.edges(sck, sck_kind) if t_lo <= t <= t_hi]
        bit_cnt = 0
        v_mosi = 0
        v_miso = 0
        word_start = None
        for te in clk_edges:
            bm = sa.level_at(mosi, te)
            bs = sa.level_at(miso, te) if miso is not None else 0
            if bit_cnt == 0:
                word_start = te
            if msb_first:
                v_mosi = (v_mosi << 1) | bm
                v_miso = (v_miso << 1) | bs
            else:
                v_mosi |= (bm << bit_cnt)
                v_miso |= (bs << bit_cnt)
            bit_cnt += 1
            if bit_cnt == bits_per_word:
                out.append(Annotation(mosi, int(word_start), int(te),
                                      f"0x{v_mosi:02X}", "byte"))
                if miso is not None:
                    out.append(Annotation(miso, int(word_start), int(te),
                                          f"0x{v_miso:02X}", "byte"))
                bit_cnt = 0
                v_mosi = 0
                v_miso = 0
        # marker khung trên kênh CS
        out.append(Annotation(cs, int(t_lo), int(t_lo), "CS", "start"))
        out.append(Annotation(cs, int(t_hi), int(t_hi), "/CS", "stop"))
    return out


# ===========================================================================
#  I2C  (SDA + SCL) — real-grade: repeated-START + clock-stretch + ACK/NACK
# ===========================================================================
def decode_i2c(sa: SampleAccessor, scl: int, sda: int) -> List[Annotation]:
    """
    Dò START (SDA↓ khi SCL cao), STOP (SDA↑ khi SCL cao). Sample SDA tại cạnh LÊN
    SCL (bám cạnh THẬT -> tự đúng khi IC kéo dài SCL = clock stretching).
    8 bit + 1 ACK/NACK. Byte đầu sau mỗi (repeated-)START = addr(7)+R/W(1).

    Truncation (capture cắt giữa giao dịch):
      - Thiếu STOP (cửa sổ kết thúc sớm): vẫn decode tới hết cửa sổ.
      - Byte cuối < 9 cạnh SCL: bỏ im lặng.
      - Thiếu START (cửa sổ BẮT ĐẦU giữa giao dịch): CHỦ Ý không decode — không
        có mốc START thì không xác định được biên bit; role accumulator sẽ nhận
        đủ giao dịch ở capture kế tiếp.
    """
    out: List[Annotation] = []

    scl_rises = sa.edges(scl, "rising")
    sda_falls = set(sa.edges(sda, "falling"))
    sda_rises = set(sa.edges(sda, "rising"))

    # START/STOP: cạnh SDA xảy ra khi SCL đang cao.
    starts: List[int] = []
    stops: List[int] = []
    for t in sorted(sda_falls | sda_rises):
        if sa.level_at(scl, t - 1) == 1 and sa.level_at(scl, t + 1) == 1:
            (starts if t in sda_falls else stops).append(t)

    if not starts:
        return out

    # Ranh giới kết thúc 1 (repeated-)START = START kế tiếp HOẶC STOP kế tiếp.
    boundaries = sorted(starts + stops)
    for t_start in starts:
        t_end = next((b for b in boundaries if b > t_start), sa.t_end)
        out.append(Annotation(sda, int(t_start), int(t_start), "S", "start"))

        bit_edges = [t for t in scl_rises if t_start < t < t_end]
        idx = 0
        first_byte = True   # byte đầu sau (repeated-)START = địa chỉ
        while idx + 9 <= len(bit_edges):
            group = bit_edges[idx:idx + 9]
            value = 0
            for k in range(8):
                value = (value << 1) | sa.level_at(sda, group[k])
            ack = sa.level_at(sda, group[8])  # 0 = ACK, 1 = NACK
            t_b0, t_b7 = group[0], group[7]
            if first_byte:
                text = f"A:0x{value >> 1:02X}{'R' if (value & 1) else 'W'}"
                first_byte = False
            else:
                text = f"0x{value:02X}"
            out.append(Annotation(sda, int(t_b0), int(t_b7), text, "byte"))
            out.append(Annotation(sda, int(group[8]), int(group[8]),
                                  "ACK" if ack == 0 else "NACK",
                                  "ack" if ack == 0 else "nack"))
            idx += 9

        if t_end in stops:
            out.append(Annotation(sda, int(t_end), int(t_end), "P", "stop"))
    return out


# ===========================================================================
#  CLOCK / PWM
# ===========================================================================
def decode_clock(sa: SampleAccessor, ch: int, is_pwm: bool = False) -> List[Annotation]:
    """1 annotation tóm tắt: tần số (+ duty nếu PWM) đặt giữa cửa sổ."""
    f = sa.fundamental_freq_hz(ch)
    if f <= 0:
        return []
    duty = sa.duty(ch) * 100.0
    text = f"{sa.format_freq(f)} {duty:.0f}%" if is_pwm else sa.format_freq(f)
    return [Annotation(ch, sa.t0, sa.t_end, text, "clock")]


# ===========================================================================
#  COUNTER  (nhóm bit -> giá trị thập phân)
# ===========================================================================
def decode_counter(sa: SampleAccessor, bit_channels: List[int],
                   max_labels: int = 300) -> List[Annotation]:
    """
    bit_channels[0] = bit0 (LSB, nhanh nhất) ... theo thứ tự tăng dần trọng số.
    Ghi giá trị thập phân mỗi khi tổ hợp đổi; đặt text trên kênh LSB.
    """
    out: List[Annotation] = []
    if not bit_channels:
        return out
    lsb = bit_channels[0]

    change_ticks = set()
    for ch in bit_channels:
        for i in sa.transitions(ch):
            change_ticks.add(sa.x_axis[i])
    if not change_ticks:
        return out
    ticks = sorted(change_ticks)

    def value_at(t):
        v = 0
        for w, ch in enumerate(bit_channels):
            v |= (sa.level_at(ch, t) << w)
        return v

    prev_val = None
    for j, t in enumerate(ticks):
        val = value_at(t + 1)  # mức ngay sau cạnh
        if val != prev_val:
            t_next = ticks[j + 1] if j + 1 < len(ticks) else sa.t_end
            out.append(Annotation(lsb, int(t), int(t_next), str(val), "counter"))
            prev_val = val
        if len(out) >= max_labels:
            break
    return out


# ===========================================================================
#  Bộ điều phối
# ===========================================================================
def decode_all(sa: SampleAccessor, roles, uart_baud: Optional[int] = None,
               uart_data_bits: int = 8, uart_parity: str = "N",
               uart_stop_bits: int = 1) -> List[Annotation]:
    """Chạy toàn bộ decoder tương ứng với vai trò các kênh."""
    out: List[Annotation] = []
    groups = _groups(roles)

    # --- UART (mỗi kênh độc lập) ---
    for ch in _by_role(roles, "UART"):
        out.extend(decode_uart(sa, ch, baud=uart_baud, data_bits=uart_data_bits,
                               parity=uart_parity, stop_bits=uart_stop_bits))

    # --- Clock / PWM ---
    for ch in _by_role(roles, "CLOCK"):
        out.extend(decode_clock(sa, ch, is_pwm=False))
    for ch in _by_role(roles, "PWM"):
        out.extend(decode_clock(sa, ch, is_pwm=True))

    # --- SPI (theo group; MISO tùy chọn) ---
    _decode_bus(out, groups, roles,
                need={"SPI_SCK": "sck", "SPI_MOSI": "mosi", "SPI_CS": "cs"},
                optional={"SPI_MISO": "miso"},
                fn=lambda a: decode_spi(sa, a["sck"], a["mosi"], a["cs"],
                                        miso=a.get("miso")))

    # --- I2C (theo group) ---
    _decode_bus(out, groups, roles,
                need={"I2C_SCL": "scl", "I2C_SDA": "sda"},
                optional={},
                fn=lambda a: decode_i2c(sa, a["scl"], a["sda"]))

    # --- Counter (gom mọi COUNTER_BIT theo group, sắp theo thứ tự bit) ---
    for gid, members in groups.items():
        bits = [r for r in members if r.role == "COUNTER_BIT"]
        if not bits:
            continue
        if all("-b" in (getattr(r, "label", "") or "") for r in bits):
            bits.sort(key=_bit_index)
        else:
            bits.sort(key=lambda r: sa.num_transitions(r.ch), reverse=True)
        out.extend(decode_counter(sa, [r.ch for r in bits]))

    return out


def _bit_index(role) -> int:
    """Lấy chỉ số bit từ label 'counter-bN' nếu có, ngược lại theo ch."""
    lbl = getattr(role, "label", "") or ""
    if "-b" in lbl:
        try:
            return int(lbl.split("-b")[-1])
        except ValueError:
            pass
    return role.ch


def _decode_bus(out, groups, roles, need: Dict[str, str], fn,
                optional: Optional[Dict[str, str]] = None):
    """
    Gom các bus (SPI/I2C) theo group_id. `need` = role bắt buộc, `optional` =
    role tùy chọn (VD SPI_MISO). Nếu roles không gán group_id thì thử ghép 1 bus.
    """
    optional = optional or {}
    matched_any = False
    for gid, members in groups.items():
        if gid is None:
            continue
        args = {}
        for role_name, key in need.items():
            found = [m for m in members if m.role == role_name]
            if found:
                args[key] = found[0].ch
        if len(args) < len(need):
            continue
        for role_name, key in optional.items():
            found = [m for m in members if m.role == role_name]
            if found:
                args[key] = found[0].ch
        out.extend(fn(args))
        matched_any = True

    if not matched_any:
        # fallback: ghép 1 bus từ toàn bộ roles (group_id=None)
        args = {}
        for role_name, key in need.items():
            found = _by_role(roles, role_name)
            if found:
                args[key] = found[0]
        if len(args) == len(need):
            for role_name, key in optional.items():
                found = _by_role(roles, role_name)
                if found:
                    args[key] = found[0]
            out.extend(fn(args))
