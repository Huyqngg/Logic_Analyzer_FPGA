"""
channel_predictor.py  (P3)
==========================
Tự động ĐOÁN vai trò của 16 kênh -- PERMUTATION-INVARIANT (độc lập vị trí dây).

Nguyên tắc: mọi bước quét theo ĐẶC TRƯNG / QUAN HỆ THỜI GIAN giữa các tín hiệu,
KHÔNG theo chỉ số kênh, KHÔNG cần các dây cùng cụm đứng cạnh nhau. Bảng ground-truth
(A.1) chỉ để chấm điểm, tuyệt đối KHÔNG hardcode vào thuật toán.

Bản v3 "realistic-capture": chạy đúng với capture PHẦN CỨNG THẬT, nơi buffer RLE
8192 record bị các kênh clock liên tục lấp đầy sau ~2.66ms < 1 vòng burst ESP32
=> mỗi cửa sổ chỉ chứa TỐI ĐA 1 burst mỗi giao thức (có thể cụt / vắng mặt):
    - CLOCK/PWM xét TRƯỚC nhưng chỉ nhận tín hiệu LIÊN TỤC phủ cửa sổ
      (_continuity) -> SCL burst không bao giờ thành "pwm-101kHz-96%" nữa.
    - I2C không còn đòi is_gated; bằng chứng = decode thật (ACK hoặc NACK,
      chấp nhận giao dịch cụt cuối).
    - SPI nhận từ 1 khung CS duy nhất (>=8 cạnh SCK trong khung, >=1 byte).
    - UART cần khung SẠCH (ổn định bit-cell) + giải thích được >=60% số cạnh
      của chính kênh -> I2C SDA không giả UART được nữa.
    - RoleAccumulator: gộp niềm tin qua NHIỀU capture (Auto Run) -> cửa sổ
      thiếu SPI sẽ được cửa sổ sau bù; cửa sổ rỗng không hạ cấp kết quả cũ.

Mỗi kênh nhận đúng 1 vai trò kèm confidence + evidence. Rule-based vì GIẢI THÍCH
ĐƯỢC. Không import tkinter/matplotlib -> test offline được.
"""

from __future__ import annotations

from dataclasses import dataclass
from statistics import median
from typing import Dict, List, Optional, Tuple

from sample_access import SampleAccessor
from decoders import decode_i2c, uart_frame_quality


@dataclass
class ChannelRole:
    ch: int
    role: str
    label: str
    group_id: Optional[int] = None
    confidence: float = 0.0
    evidence: Optional[dict] = None


# Ngưỡng
MIN_TRANSITIONS = 4          # dưới mức này coi là IDLE (bước quét CUỐI)
IDLE_DEAD_TRANSITIONS = 2    # bước ĐẦU chỉ loại kênh chết hẳn: CS 1 khung chỉ có
                             # 2 transition, phải giữ lại cho SPI search
SQUARE_DUTY_LO, SQUARE_DUTY_HI = 0.30, 0.70
CLOCK_DUTY_LO, CLOCK_DUTY_HI = 0.42, 0.58
RATIO_LO, RATIO_HI = 1.6, 2.5   # cửa sổ tỉ lệ 2:1 cho chuỗi counter
I2C_MIN_SCORE = 2.5
SPI_MIN_SCORE = 0.5          # >=1 byte decode được (1 khung CS duy nhất vẫn nhận)
UART_MIN_FRAMES = 2          # cần >=2 frame hợp lệ -> tránh nhầm SDA/nhiễu thành UART
UART_MIN_EDGE_EXPL = 0.6     # khung UART phải giải thích >=60% số cạnh của kênh
STRONG_OVERWRITE_MARGIN = 0.15   # hysteresis khi role mạnh ghi đè role mạnh khác
CONTINUITY_SPAN_FRAC = 0.30      # slack độ phủ cửa sổ cho tín hiệu liên tục
CONTINUITY_PERIOD_SLACK = 3      # ... hoặc 3 chu kỳ (tín hiệu chậm: PWM 1kHz)
MIN_CONT_PERIODS = 1.8           # CLOCK/PWM phải thấy >=1.8 chu kỳ trong cửa sổ
DUTY_EXTREME_LO, DUTY_EXTREME_HI = 0.03, 0.97  # duty ngoài khoảng này ~ DC/blip
                                 # (thường là CS/glitch) -> không nhận PWM


# ===========================================================================
def predict_roles(sa: SampleAccessor, uart_baud: Optional[int] = None) -> List[ChannelRole]:
    n = sa.num_channels
    roles: List[ChannelRole] = [ChannelRole(c, "UNKNOWN", "unknown") for c in range(n)]
    available = set(range(n))
    group_seq = 0

    def assign(ch, role, label, gid=None, conf=0.5, evidence=None):
        roles[ch] = ChannelRole(ch, role, label, gid, conf, evidence)
        available.discard(ch)

    # -------- 1. IDLE (chỉ kênh CHẾT HẲN) -----------------------------------
    #  Không dùng MIN_TRANSITIONS ở đây: CS của 1 khung SPI duy nhất chỉ có
    #  2 transition -> phải sống sót tới bước SPI. Kênh ít hoạt động còn lại
    #  sẽ được gán IDLE ở bước 7.
    for c in list(available):
        if sa.num_transitions(c) < IDLE_DEAD_TRANSITIONS:
            assign(c, "IDLE", "idle", conf=0.9)

    # -------- 2. COUNTER (chuỗi 2:1, >= 3 bit) -----------------------------
    #  Trước CLOCK/PWM để chuỗi 2:1 không bị CLOCK nuốt từng bit.
    chain = _best_counter_chain(sa, available)
    if chain:
        group_seq += 1
        for bit_idx, ch in enumerate(chain):
            assign(ch, "COUNTER_BIT", f"counter-b{bit_idx}", group_seq, 0.8)

    # -------- 3. CLOCK / PWM (CHỈ tín hiệu LIÊN TỤC) ------------------------
    #  Xét trước các bus: rút kênh liên tục khỏi pool -> thu hẹp không gian
    #  tìm I2C/SPI và chặn đường "SCL burst -> PWM".
    for c in list(available):
        info = _continuity(sa, c)
        if info is None:
            continue
        duty = info["duty"]
        f = info["freq"]
        if CLOCK_DUTY_LO <= duty <= CLOCK_DUTY_HI:
            assign(c, "CLOCK", f"clk-{sa.format_freq(f)}", conf=0.85, evidence=info)
        else:
            assign(c, "PWM", f"pwm-{sa.format_freq(f)}-{duty*100:.0f}%",
                   conf=0.8, evidence=info)

    # -------- 4. I2C (nhiều cặp SDA/SCL) -----------------------------------
    while True:
        best = _best_i2c_pair(sa, available)
        if not best:
            break
        scl, sda, score, ev = best
        group_seq += 1
        conf = min(0.99, 0.5 + score * 0.05)
        assign(scl, "I2C_SCL", "i2c-scl", group_seq, conf, evidence=ev)
        assign(sda, "I2C_SDA", "i2c-sda", group_seq, conf, evidence=ev)

    # -------- 5. SPI (nhiều bộ CS/SCK/MOSI [+MISO]) ------------------------
    while True:
        spi = _best_spi_triple(sa, available)
        if not spi:
            break
        cs, sck, mosi, score, ev = spi
        group_seq += 1
        conf = min(0.99, 0.5 + score * 0.05)
        assign(cs, "SPI_CS", "spi-cs", group_seq, conf, evidence=ev)
        assign(sck, "SPI_SCK", "spi-sck", group_seq, conf, evidence=ev)
        assign(mosi, "SPI_MOSI", "spi-mosi", group_seq, conf, evidence=ev)
        # MISO: đường data thứ 2 trong khung CS (chiều slave->master)
        miso = _find_spi_miso(sa, available, cs, sck)
        if miso is not None:
            assign(miso, "SPI_MISO", "spi-miso", group_seq, conf * 0.9, evidence=ev)

    # -------- 6. UART (idle-high, khung SẠCH + giải thích đủ số cạnh) ------
    for c in list(available):
        if sa.idle_level(c) != 1:
            continue
        frames, edge_expl, bit_ticks = uart_frame_quality(sa, c, baud=uart_baud)
        if len(frames) >= UART_MIN_FRAMES and edge_expl >= UART_MIN_EDGE_EXPL:
            # Chống alias clock: chuỗi toggle thuần (SCL/SCK của bus không được
            # nhận) decode ra toàn 0x55/0xAA "hợp lệ" -> không dám nhận là UART.
            vals = {int(f.text[2:4], 16) for f in frames if f.kind == "byte"}
            if vals and vals <= {0x55, 0xAA}:
                continue
            conf = min(0.99, 0.4 + 0.05 * len(frames) + 0.2 * edge_expl)
            assign(c, "UART", "uart-data", conf=conf,
                   evidence={"frames": len(frames), "edge_expl": round(edge_expl, 2),
                             "bit_ticks": round(bit_ticks, 1)})

    # -------- 7. Còn lại -> UNKNOWN/IDLE -----------------------------------
    for c in list(available):
        if sa.num_transitions(c) < MIN_TRANSITIONS:
            assign(c, "IDLE", "idle", conf=0.6)
        else:
            assign(c, "UNKNOWN", "unknown", conf=0.2)

    return roles


# ===========================================================================
#  CLOCK / PWM: yêu cầu LIÊN TỤC (chống nhầm burst giao thức thành clock)
# ===========================================================================
def _continuity(sa: SampleAccessor, ch: int) -> Optional[dict]:
    """
    None nếu kênh KHÔNG phải tín hiệu tuần hoàn liên tục; ngược lại trả
    {'freq','duty','periods','coverage'}.
      - Không gated (burst + idle dài -> không phải clock).
      - Tuần hoàn: CV chu kỳ < 0.30, HOẶC đường "PWM chậm" (ít chu kỳ trong
        cửa sổ, vd 1kHz @2.66ms ~ 2.6 chu kỳ): độ rộng run nội mỗi mức rất đều.
      - Cạnh phải trải gần hết cửa sổ (chừa slack 30% hoặc 3 chu kỳ).
      - duty đo bằng duty_active (không bị idle biên pha loãng).
    """
    if sa.num_transitions(ch) < MIN_TRANSITIONS:
        return None
    if sa.is_gated(ch):
        return None

    period = 0.0
    rises = sa.edges(ch, "rising")
    if len(rises) >= 3 and _period_cv(sa, ch) < 0.30:
        period = sa.fundamental_period_ticks(ch)
    else:
        # PWM chậm: đo bằng độ rộng run nội (đã bỏ run biên bị cắt cụt).
        hi_ws = sa.run_widths(ch, 1, trim_boundary=True)
        lo_ws = sa.run_widths(ch, 0, trim_boundary=True)
        if len(sa.runs(ch)) >= 4 and hi_ws and lo_ws:
            def _tight(ws):
                if len(ws) == 1:
                    return True
                m = median(ws)
                return m > 0 and (max(ws) - min(ws)) / m < 0.25
            if _tight(hi_ws) and _tight(lo_ws):
                period = float(median(hi_ws) + median(lo_ws))
    if period <= 0:
        return None

    es = sa.edges(ch, "both")
    span = es[-1] - es[0]
    slack = max(CONTINUITY_SPAN_FRAC * sa.duration_ticks,
                CONTINUITY_PERIOD_SLACK * period)
    if span < sa.duration_ticks - slack:
        return None

    # Phải thấy đủ số chu kỳ: 2 khung CS cách nhau 1 vòng loop nhìn như "PWM
    # 1.2 chu kỳ" (slack 3-chu-kỳ ở trên thành vô nghĩa khi period ~ cửa sổ).
    # PWM chậm nhất của hệ (1kHz) luôn có >=2.6 chu kỳ trong buffer 8192 record.
    if sa.duration_ticks / period < MIN_CONT_PERIODS:
        return None

    # Duty: ưu tiên median run NỘI mỗi mức — độc lập pha cửa sổ. Capture thật
    # 1kHz chỉ ~2.6 chu kỳ: duty theo span segment lệch thành 60%/67% thay vì 75%.
    hi_ws = sa.run_widths(ch, 1, trim_boundary=True)
    lo_ws = sa.run_widths(ch, 0, trim_boundary=True)
    if hi_ws and lo_ws and (median(hi_ws) + median(lo_ws)) > 0:
        duty = median(hi_ws) / (median(hi_ws) + median(lo_ws))
    else:
        duty = sa.duty_active(ch)
    if not (DUTY_EXTREME_LO <= duty <= DUTY_EXTREME_HI):
        return None
    return {"freq": sa.sample_rate_hz / period, "duty": duty,
            "periods": sa.duration_ticks / period,
            "coverage": (span / sa.duration_ticks) if sa.duration_ticks else 0.0}


# ===========================================================================
#  I2C pair search  (diệt false-positive bằng ràng buộc bất biến)
# ===========================================================================
def _sda_scl_edge_split(sa: SampleAccessor, scl: int, sda: int):
    """Đếm cạnh SDA khi SCL đang CAO vs đang THẤP.
    I2C thật: data đổi khi SCL THẤP (n_low lớn); START/STOP khi SCL cao (n_high nhỏ).
    PWM/clock giả làm SDA: cạnh rơi ngẫu nhiên -> n_high lớn -> loại.
    """
    n_high = n_low = 0
    for t in sa.edges(sda, "both"):
        hi_before = sa.level_at(scl, t - 1) == 1
        hi_after = sa.level_at(scl, t + 1) == 1
        if hi_before and hi_after:
            n_high += 1
        elif (not hi_before) and (not hi_after):
            n_low += 1
    return n_high, n_low


def _i2c_score(sa: SampleAccessor, scl: int, sda: int) -> Tuple[float, Optional[dict]]:
    """Điểm cho giả thuyết (scl, sda) + evidence. Trả (0, None) nếu vi phạm.

    KHÔNG đòi is_gated: capture thật có thể chỉ chứa 1 giao dịch (thậm chí cụt);
    bằng chứng quyết định là decode thật bên dưới.
    """
    # SCL: clock idle-high, đủ nhịp để thăm dò >=1 byte (có thể cụt).
    if sa.idle_level(scl) != 1 or sa.num_transitions(scl) < 10:
        return 0.0, None
    # SDA: idle-high.
    if sa.idle_level(sda) != 1:
        return 0.0, None

    # Ràng buộc bất biến: cạnh SDA phải chủ yếu xảy ra khi SCL THẤP.
    n_high, n_low = _sda_scl_edge_split(sa, scl, sda)
    if n_high > n_low:            # PWM/clock/data ngẫu nhiên -> loại
        return 0.0, None

    anns = decode_i2c(sa, scl, sda)
    if not anns:
        return 0.0, None
    starts = sum(1 for a in anns if a.kind == "start")
    byte_anns = [a for a in anns if a.kind == "byte"]
    acks = sum(1 for a in anns if a.kind in ("ack", "nack"))   # chấp nhận cả NACK
    if starts == 0 or not byte_anns or acks == 0:
        return 0.0, None
    # LƯU Ý: KHÔNG veto khi bytes < starts. Phần cứng thật (100MHz) phân giải được
    # cạnh SDA lúc SCL cao ở pha nhả bus/turnaround -> sinh START "thừa" không có
    # byte theo sau (dump t8r_s0: starts=3, bytes=2 nhưng addr chuẩn A:0x3CW/R,
    # veto làm SCL rơi xuống UART-0x55). START thừa đã bị trừ điểm trong score.

    # Thưởng "địa chỉ nhất quán": IC/ESP32 poll lặp lại cùng địa chỉ.
    addrs = [a.text for a in byte_anns if a.text.startswith("A:")]
    addr_bonus = 0.0
    if addrs:
        most = max(set(addrs), key=addrs.count)
        addr_bonus = 3.0 * addrs.count(most) / len(addrs)

    score = max(0.0, 2 * len(byte_anns) + acks - starts + addr_bonus)
    ev = {"bytes": len(byte_anns), "acks": acks, "starts": starts,
          "score": round(score, 1)}
    return score, ev


def _best_i2c_pair(sa: SampleAccessor, available):
    """Duyệt MỌI cặp trong `available`, thử 2 chiều, chọn cặp điểm cao nhất."""
    cands = list(available)
    best = None
    best_score = I2C_MIN_SCORE
    for a in cands:
        for b in cands:
            if a == b:
                continue
            s, ev = _i2c_score(sa, a, b)   # a=SCL, b=SDA
            if s > best_score:
                best_score = s
                best = (a, b, s, ev)
    return best


# ===========================================================================
#  SPI triple + MISO search  (nhận được từ 1 khung CS duy nhất)
# ===========================================================================
def _frames_of_cs(sa: SampleAccessor, cs: int):
    falls = sa.edges(cs, "falling")
    rises = sa.edges(cs, "rising")
    frames = []
    # Khung cụt đầu: CS đã thấp ngay đầu cửa sổ.
    if sa.level_at(cs, sa.t0) == 0 and rises:
        if not falls or rises[0] < falls[0]:
            frames.append((sa.t0, rises[0]))
    for tf in falls:
        tr = next((t for t in rises if t > tf), sa.t_end)
        frames.append((tf, tr))
    return frames


def _edges_in_frames(edges, frames):
    inside = outside = 0
    for t in edges:
        if any(lo <= t <= hi for (lo, hi) in frames):
            inside += 1
        else:
            outside += 1
    return inside, outside


def _spi_score(sa: SampleAccessor, cs: int, sck: int, mosi: int) -> float:
    from decoders import decode_spi
    anns = decode_spi(sa, sck, mosi, cs)
    return float(sum(1 for a in anns if a.kind == "byte"))


def _best_spi_triple(sa: SampleAccessor, available):
    cands = list(available)
    # CS: idle-high, có khung thấp. Capture thật: 1 khung 40us trong 2.66ms
    # -> duty ~0.985; khung cụt đầu có thể kéo duty thấp -> chỉ chặn dưới.
    cs_cands = [c for c in cands
                if sa.idle_level(c) == 1 and sa.duty(c) > 0.4
                and len(sa.edges(c, "falling")) >= 1]
    best = None
    best_score = SPI_MIN_SCORE
    for cs in cs_cands:
        frames = _frames_of_cs(sa, cs)
        if not frames:
            continue
        # SCK: nhiều cạnh lên, tập trung TRONG khung CS-low (gated theo CS).
        # >=8 cạnh lên trong khung = trọn 1 byte -> 1 khung CS duy nhất vẫn nhận.
        sck_cands = []
        for c in cands:
            if c == cs:
                continue
            rises = sa.edges(c, "rising")
            if len(rises) < 8:
                continue
            inside, outside = _edges_in_frames(rises, frames)
            if inside >= 8 and inside >= 3 * (outside + 1):
                sck_cands.append((inside, c))
        sck_cands.sort(reverse=True)
        for sck_inside, sck in sck_cands[:3]:
            for mosi in cands:
                if mosi in (cs, sck):
                    continue
                m_edges = sa.edges(mosi, "both")
                if not m_edges:
                    continue
                inside, outside = _edges_in_frames(m_edges, frames)
                if inside < outside:
                    continue
                score = _spi_score(sa, cs, sck, mosi)
                if score > best_score:
                    best_score = score
                    best = (cs, sck, mosi, score,
                            {"bytes": int(score), "sck_inside": sck_inside})
    return best


def _find_spi_miso(sa: SampleAccessor, available, cs: int, sck: int):
    """Tìm đường data thứ 2 (MISO) trong `available`: đổi mức GẦN NHƯ CHỈ trong
    khung CS, khác SCK. Không phân biệt được MOSI/MISO theo hướng vật lý -> gán
    ứng viên còn lại; người dùng đổi tay được (Decode Setup). MISO không jumper
    (nằm im) -> < 2 cạnh -> None."""
    frames = _frames_of_cs(sa, cs)
    if not frames:
        return None
    best = None
    best_inside = 0
    for c in list(available):
        if c in (cs, sck):
            continue
        edges = sa.edges(c, "both")
        if len(edges) < 2:
            continue
        inside, outside = _edges_in_frames(edges, frames)
        # outside phải rất nhỏ: kênh UART chỉ "quẹt" qua khung sẽ có outside lớn.
        if inside >= 2 and outside <= inside // 3 and inside > best_inside:
            best_inside = inside
            best = c
    return best


# ===========================================================================
#  Counter chain search (>= 3 bit, tỉ lệ 2:1)
# ===========================================================================
def _is_periodic_square(sa: SampleAccessor, ch: int, strict: bool) -> bool:
    if sa.num_transitions(ch) < MIN_TRANSITIONS:
        return False
    duty = sa.duty(ch)
    if not (SQUARE_DUTY_LO <= duty <= SQUARE_DUTY_HI):
        return False
    if _continuity(sa, ch) is None:      # burst/gated -> không phải counter bit
        return False
    cv = sa.coefficient_of_variation(ch)
    return cv < (0.35 if strict else 0.6)


def _period_cv(sa: SampleAccessor, ch: int) -> float:
    r = sa.edges(ch, "rising")
    if len(r) < 3:
        return 1.0
    diffs = [r[i + 1] - r[i] for i in range(len(r) - 1)]
    m = sum(diffs) / len(diffs)
    if m == 0:
        return 1.0
    var = sum((d - m) ** 2 for d in diffs) / len(diffs)
    return (var ** 0.5) / m


def _is_periodic(sa: SampleAccessor, ch: int) -> bool:
    if sa.num_transitions(ch) < MIN_TRANSITIONS:
        return False
    return _period_cv(sa, ch) < 0.30


def _best_counter_chain(sa: SampleAccessor, available):
    cands = []
    for c in available:
        if not _is_periodic_square(sa, c, strict=False):
            continue
        p = sa.fundamental_period_ticks(c)
        if p > 0:
            cands.append((p, c))
    if len(cands) < 3:
        return None
    cands.sort()

    best_chain: List = []
    for start in range(len(cands)):
        chain = [cands[start]]
        cur_p = cands[start][0]
        for k in range(start + 1, len(cands)):
            p, c = cands[k]
            ratio = p / cur_p
            if RATIO_LO <= ratio <= RATIO_HI:
                chain.append((p, c))
                cur_p = p
        if len(chain) > len(best_chain):
            best_chain = chain

    if len(best_chain) >= 3:
        return [c for (_p, c) in best_chain]
    return None


# ===========================================================================
#  RoleAccumulator: niềm tin vai trò BỀN VỮNG qua nhiều capture (Auto Run)
# ===========================================================================
WEAK_ROLES = {"IDLE", "UNKNOWN"}


class RoleAccumulator:
    """
    Cửa sổ capture (~2.66ms) < 1 vòng burst ESP32 -> mỗi capture chỉ thấy một
    phần giao thức. Gộp dự đoán của nhiều cửa sổ để hội tụ đủ 16 kênh:
      - IDLE/UNKNOWN không bao giờ hạ cấp role mạnh đã có (cửa sổ trượt burst
        không phá kết quả đúng trước đó).
      - Cùng role -> refresh (conf=max, label/evidence mới nhất).
      - Khác role mạnh -> cần conf mới >= conf cũ + STRONG_OVERWRITE_MARGIN
        (hysteresis chống nhấp nháy giữa các cửa sổ).
      - Bus (I2C/SPI/COUNTER, có group_id) ghi NGUYÊN TỬ cả nhóm: không bao giờ
        cướp SCL mà bỏ lại SDA cũ.
      - set_manual: người dùng gán tay -> khóa kênh tới khi reset.
    """

    def __init__(self, num_channels: int = 16):
        self.num_channels = num_channels
        self.reset()

    def reset(self) -> None:
        self._roles: List[Optional[ChannelRole]] = [None] * self.num_channels
        self._locked: List[bool] = [False] * self.num_channels
        self._next_gid: int = 0

    def set_manual(self, roles: List[ChannelRole]) -> None:
        for r in roles:
            if r.ch < self.num_channels:
                self._roles[r.ch] = r
                self._locked[r.ch] = True

    def current(self) -> Optional[List[ChannelRole]]:
        if all(r is None for r in self._roles):
            return None
        return [self._roles[c] if self._roles[c] is not None
                else ChannelRole(c, "UNKNOWN", "unknown")
                for c in range(self.num_channels)]

    # ------------------------------------------------------------- merge
    def _can_write(self, ch: int, new: ChannelRole) -> bool:
        if self._locked[ch]:
            return False
        old = self._roles[ch]
        if new.role in WEAK_ROLES:
            return old is None or old.role in WEAK_ROLES
        if old is None or old.role in WEAK_ROLES:
            return True
        if old.role == new.role:
            return True
        return new.confidence >= old.confidence + STRONG_OVERWRITE_MARGIN

    def _store(self, new: ChannelRole, gid: Optional[int]) -> None:
        old = self._roles[new.ch]
        conf = new.confidence
        if old is not None and old.role == new.role:
            conf = max(old.confidence, new.confidence)
        self._roles[new.ch] = ChannelRole(new.ch, new.role, new.label,
                                          gid, conf, new.evidence)

    def _persistent_gid(self, members) -> int:
        """gid dự đoán reset về 1 mỗi cửa sổ -> ánh xạ sang namespace bền vững.
        Tái dùng gid cũ nếu đúng bộ (kênh, role) này đã tồn tại."""
        chans = {m.ch for m in members}
        new_roleset = {(m.ch, m.role) for m in members}
        old_gids = {self._roles[c].group_id for c in chans
                    if self._roles[c] is not None
                    and self._roles[c].group_id is not None}
        if len(old_gids) == 1:
            gid = next(iter(old_gids))
            old_roleset = {(c, self._roles[c].role)
                           for c in range(self.num_channels)
                           if self._roles[c] is not None
                           and self._roles[c].group_id == gid}
            if old_roleset == new_roleset:
                return gid
        self._next_gid += 1
        return self._next_gid

    def observe(self, predicted: List[ChannelRole]) -> List[ChannelRole]:
        """Gộp 1 lượt dự đoán vào niềm tin; trả danh sách role hiện hành."""
        units: Dict[int, List[ChannelRole]] = {}
        singles: List[ChannelRole] = []
        for r in predicted:
            if r.group_id is not None:
                units.setdefault(r.group_id, []).append(r)
            else:
                singles.append(r)

        # Bus trước (ưu tiên nhóm tự tin nhất), ghi NGUYÊN TỬ.
        for _gid, members in sorted(units.items(),
                                    key=lambda kv: -min(m.confidence for m in kv[1])):
            if any(self._locked[m.ch] for m in members):
                continue
            if not all(self._can_write(m.ch, m) for m in members):
                continue
            gid = self._persistent_gid(members)
            for m in members:
                self._store(m, gid)

        # Kênh đơn (UART/CLOCK/PWM/IDLE/UNKNOWN).
        for r in singles:
            if self._can_write(r.ch, r):
                self._store(r, None)

        cur = self.current()
        return cur if cur is not None else list(predicted)


# ===========================================================================
#  Tiện ích
# ===========================================================================
def accuracy_vs_groundtruth(roles: List[ChannelRole], truth: dict) -> float:
    if not truth:
        return 0.0
    ok = sum(1 for r in roles if truth.get(r.ch) == r.role)
    return ok / len(truth)
