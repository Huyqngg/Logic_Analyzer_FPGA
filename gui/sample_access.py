"""
sample_access.py  (P3)
======================
Lớp truy cập mẫu dùng chung cho predictor + decoder.

Bọc dữ liệu thô mà `gui/main.py` đã dựng sẵn:
    - x_axis        : list tick TÍCH LŨY (đơn điệu tăng) - mỗi phần tử là 1 "event"
                      mà FPGA ghi ra khi BẤT KỲ kênh nào đổi mức (union mọi transition).
    - channel_data  : list[num_channels] các list mức 0/1 tại từng event tương ứng.
    - scaler_val    : = (scaler + 1). 1 tick = scaler_val * 10 ns  (clock hệ thống 100MHz).

Vì `x_axis` là HỢP của mọi transition trên mọi kênh, nên tra mức của một kênh bất kỳ
tại một tick bất kỳ là chính xác (không phụ thuộc kênh nào ở vị trí nào)
    level_at(ch, tick) = channel_data[ch][bisect_right(x_axis, tick) - 1]

Đây là cơ sở cho tính "PERMUTATION-INVARIANT": decode chỉ cần biết vai trò kênh,
không cần biết kênh nằm ở vị trí nào.

Module này KHÔNG import tkinter/matplotlib -> unit-test được offline.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass, field
from statistics import median
from typing import List, Optional, Tuple

# Clock hệ thống FPGA: 100 MHz -> 10 ns / chu kỳ. 1 tick = scaler_val * 10 ns.
NS_PER_BASE_TICK = 10.0


@dataclass
class Run:
    """Một đoạn giữ nguyên mức của 1 kênh: [t_start, t_end) mang mức `level`."""
    t_start: int
    t_end: int
    level: int

    @property
    def width(self) -> int:
        return self.t_end - self.t_start


class SampleAccessor:
    """API chung cho mọi predictor/decoder. Làm việc hoàn toàn trong MIỀN TICK."""

    def __init__(self, x_axis, channel_data, scaler_val: int, num_channels: int = 16):
        # Copy nông sang tuple/list để tránh bị thread capture sửa giữa chừng.
        self.x_axis: List[int] = list(x_axis)
        self.channel_data: List[List[int]] = [list(channel_data[c]) for c in range(num_channels)]
        self.scaler_val: int = max(1, int(scaler_val))
        self.num_channels: int = num_channels

        self.n: int = len(self.x_axis)
        self.t0: int = self.x_axis[0] if self.n else 0
        self.t_end: int = self.x_axis[-1] if self.n else 0

        # cache
        self._transitions: List[Optional[List[int]]] = [None] * num_channels  # index của event đổi mức
        self._runs: List[Optional[List[Run]]] = [None] * num_channels
        self._segments: List[Optional[List[Tuple[int, int]]]] = [None] * num_channels

    # ----------------------------------------------------------------- time
    @property
    def duration_ticks(self) -> int:
        return self.t_end - self.t0

    @property
    def sample_rate_hz(self) -> float:
        """Tần số lấy mẫu thực tế = 100MHz / scaler_val."""
        return 1e9 / (self.scaler_val * NS_PER_BASE_TICK)

    def tick_to_ns(self, ticks: float) -> float:
        return ticks * self.scaler_val * NS_PER_BASE_TICK

    def ns_to_ticks(self, ns: float) -> float:
        return ns / (self.scaler_val * NS_PER_BASE_TICK)

    def hz_to_ticks(self, hz: float) -> float:
        """Số tick của 1 chu kỳ tần số `hz`."""
        return self.sample_rate_hz / hz if hz else 0.0

    # ---------------------------------------------------------------- level
    def _index_at(self, tick: float) -> int:
        idx = bisect_right(self.x_axis, tick) - 1
        if idx < 0:
            idx = 0
        elif idx >= self.n:
            idx = self.n - 1
        return idx

    def level_at(self, ch: int, tick: float) -> int:
        if self.n == 0:
            return 0
        return self.channel_data[ch][self._index_at(tick)]

    # ----------------------------------------------------------- transitions
    def transitions(self, ch: int) -> List[int]:
        """Index (trong x_axis) của các event mà kênh `ch` ĐỔI mức so với event trước."""
        cached = self._transitions[ch]
        if cached is not None:
            return cached
        data = self.channel_data[ch]
        out: List[int] = []
        for i in range(1, self.n):
            if data[i] != data[i - 1]:
                out.append(i)
        self._transitions[ch] = out
        return out

    def edges(self, ch: int, kind: str = "both") -> List[int]:
        """
        Trả về TICK của các cạnh.
          kind='rising' : cạnh lên (0->1)
          kind='falling': cạnh xuống (1->0)
          kind='both'   : mọi cạnh
        """
        data = self.channel_data[ch]
        x = self.x_axis
        out: List[int] = []
        for i in self.transitions(ch):
            if kind == "rising" and data[i] == 1:
                out.append(x[i])
            elif kind == "falling" and data[i] == 0:
                out.append(x[i])
            elif kind == "both":
                out.append(x[i])
        return out

    def num_transitions(self, ch: int) -> int:
        return len(self.transitions(ch))

    # ------------------------------------------------------------------ runs
    def runs(self, ch: int) -> List[Run]:
        """Danh sách các đoạn giữ mức của kênh (nén RLE trong miền tick)."""
        cached = self._runs[ch]
        if cached is not None:
            return cached
        data = self.channel_data[ch]
        x = self.x_axis
        out: List[Run] = []
        if self.n == 0:
            self._runs[ch] = out
            return out
        seg_start = x[0]
        seg_level = data[0]
        for i in range(1, self.n):
            if data[i] != seg_level:
                out.append(Run(seg_start, x[i], seg_level))
                seg_start = x[i]
                seg_level = data[i]
        out.append(Run(seg_start, x[-1], seg_level))
        self._runs[ch] = out
        return out

    # -------------------------------------------------------------- features
    def idle_level(self, ch: int) -> int:
        """Mức nghỉ = mức chiếm nhiều thời gian nhất (time-weighted)."""
        hi = lo = 0
        for r in self.runs(ch):
            if r.level:
                hi += r.width
            else:
                lo += r.width
        return 1 if hi >= lo else 0

    def duty(self, ch: int) -> float:
        """Tỉ lệ thời gian ở mức cao."""
        hi = tot = 0
        for r in self.runs(ch):
            tot += r.width
            if r.level:
                hi += r.width
        return (hi / tot) if tot else 0.0

    def activity(self, ch: int) -> float:
        """Mật độ chuyển mức trên toàn cửa sổ (transition / tick)."""
        return self.num_transitions(ch) / self.duration_ticks if self.duration_ticks else 0.0

    def run_widths(self, ch: int, level: Optional[int] = None,
                   trim_boundary: bool = False) -> List[int]:
        """
        Độ rộng các run. Mặc định GIỮ TẤT CẢ run — run idle ở 2 biên là bằng chứng
        gating quan trọng khi capture chỉ chứa 1 burst duy nhất.
        trim_boundary=True: bỏ run đầu/cuối (bị cửa sổ cắt cụt -> độ rộng không tin
        được) — dùng cho các phép đo cần độ rộng KHÔNG THIÊN LỆCH (min/CV).
        """
        rs = self.runs(ch)
        if trim_boundary and len(rs) > 2:
            rs = rs[1:-1]
        return [r.width for r in rs if (level is None or r.level == level)]

    def min_pulse_ticks(self, ch: int) -> int:
        """Độ rộng xung nhỏ nhất -> ước lượng 1 bit / nửa chu kỳ."""
        ws = self.run_widths(ch, trim_boundary=True)
        return min(ws) if ws else 0

    def fundamental_period_ticks(self, ch: int) -> float:
        """Chu kỳ cơ bản = trung vị khoảng cách giữa 2 cạnh LÊN liên tiếp."""
        r = self.edges(ch, "rising")
        if len(r) < 2:
            return 0.0
        diffs = [r[i + 1] - r[i] for i in range(len(r) - 1)]
        return float(median(diffs)) if diffs else 0.0

    def fundamental_freq_hz(self, ch: int) -> float:
        p = self.fundamental_period_ticks(ch)
        return self.sample_rate_hz / p if p else 0.0

    def coefficient_of_variation(self, ch: int, level: Optional[int] = None) -> float:
        """CV độ rộng run: nhỏ => rất đều (clock); lớn => data/gated."""
        ws = self.run_widths(ch, level, trim_boundary=True)
        if len(ws) < 2:
            return 0.0
        m = sum(ws) / len(ws)
        if m == 0:
            return 0.0
        var = sum((w - m) ** 2 for w in ws) / len(ws)
        return (var ** 0.5) / m

    # ------------------------------------------------------- burst detection
    def burst_segments(self, ch: int, gap_factor: float = 6.0) -> List[Tuple[int, int]]:
        """
        Các đoạn HOẠT ĐỘNG (t_start, t_end) của kênh: cụm cạnh liên tiếp, tách nhau
        bởi khoảng lặng > gap_factor * median(gap). Capture thật thường chỉ chứa
        1 burst/giao thức -> segment hóa để đo duty/tần số không bị idle pha loãng.
        """
        if gap_factor == 6.0 and self._segments[ch] is not None:
            return self._segments[ch]
        es = self.edges(ch, "both")
        segs: List[Tuple[int, int]] = []
        if len(es) >= 2:
            gaps = [es[i + 1] - es[i] for i in range(len(es) - 1)]
            thr = max(gap_factor * median(gaps), 2)
            clusters: List[List[int]] = [[es[0]]]
            for i, g in enumerate(gaps):
                if g > thr:
                    clusters.append([])
                clusters[-1].append(es[i + 1])
            segs = [(c[0], c[-1]) for c in clusters if len(c) >= 2]
            if not segs:
                segs = [(es[0], es[-1])]
        if gap_factor == 6.0:
            self._segments[ch] = segs
        return segs

    def activity_coverage(self, ch: int) -> float:
        """Tỉ lệ cửa sổ được các segment hoạt động che phủ (0..1)."""
        if self.duration_ticks <= 0:
            return 0.0
        segs = self.burst_segments(ch)
        return sum(e - s for (s, e) in segs) / self.duration_ticks

    def _hi_tot_in(self, ch: int, t_lo: int, t_hi: int) -> Tuple[int, int]:
        """(thời gian mức cao, tổng thời gian) của kênh trong [t_lo, t_hi]."""
        hi = tot = 0
        for r in self.runs(ch):
            lo = max(r.t_start, t_lo)
            hi_end = min(r.t_end, t_hi)
            if hi_end <= lo:
                continue
            w = hi_end - lo
            tot += w
            if r.level:
                hi += w
        return hi, tot

    def duty_in(self, ch: int, t_lo: int, t_hi: int) -> float:
        """Duty (tỉ lệ mức cao) trong khoảng [t_lo, t_hi]."""
        hi, tot = self._hi_tot_in(ch, t_lo, t_hi)
        return (hi / tot) if tot else 0.0

    def duty_active(self, ch: int) -> float:
        """
        Duty CHỈ trong các segment hoạt động — dùng cho nhãn PWM/CLOCK để idle
        dài (capture 1-burst) không kéo duty về mức nghỉ (vd '96%' giả).
        """
        segs = self.burst_segments(ch)
        if not segs:
            return self.duty(ch)
        hi = tot = 0
        for (s, e) in segs:
            h, t = self._hi_tot_in(ch, s, e)
            hi += h
            tot += t
        return (hi / tot) if tot else self.duty(ch)

    def is_gated(self, ch: int, gap_factor: float = 6.0,
                 min_idle_frac: float = 0.30, max_big_gap_frac: float = 0.25) -> bool:
        """
        Tín hiệu 'gated' = cụm xung xen kẽ những đoạn nghỉ dài (SCK/SCL/UART burst).
        Điều kiện (TÍNH CẢ idle ở 2 biên cửa sổ — capture 1-burst):
          1. Tồn tại đoạn lặng (gap trong hoặc idle biên) > gap_factor * median(gap).
          2. Tổng idle >= min_idle_frac * cửa sổ.
          3. Tỉ lệ gap-lớn <= max_big_gap_frac (chặn PWM duty cực đoan giả gated).
        """
        es = self.edges(ch, "both")
        if len(es) < 2 or self.duration_ticks <= 0:
            return False
        gaps = [es[i + 1] - es[i] for i in range(len(es) - 1)]
        thr = max(gap_factor * median(gaps), 2)

        # Idle biên được coi như gap: tín hiệu 1kHz chỉ lệch pha không tính là gated,
        # nhưng burst I2C/UART giữa 2 khoảng idle dài thì có.
        boundary_idle = [es[0] - self.t0, self.t_end - es[-1]]
        if max(gaps + boundary_idle) <= thr:
            return False

        segs = self.burst_segments(ch, gap_factor)
        active = sum(e - s for (s, e) in segs)
        idle = self.duration_ticks - active
        if idle < min_idle_frac * self.duration_ticks:
            return False

        # Guard chống PWM duty cực đoan chỉ có nghĩa khi ĐỦ nhiều gap; với ít cạnh
        # (vd CS có 2 khung SPI -> 3 gap, 1 gap lớn = 33%) nó phản tác dụng làm
        # CS "không gated" rồi bị nhận nhầm thành PWM ~300Hz duty 99%.
        big = sum(1 for g in gaps if g > thr)
        return len(gaps) < 8 or (big / len(gaps)) <= max_big_gap_frac

    # --------------------------------------------------------------- helpers
    def format_freq(self, hz: float) -> str:
        # Ngưỡng 999.5 (không phải 1000) để 999.6Hz hiển thị "1kHz"
        # thay vì "1e+03Hz" (lỗi làm tròn của định dạng .3g).
        if hz <= 0:
            return "?"
        if hz >= 999.5e3:
            return f"{hz / 1e6:.3g}MHz"
        if hz >= 999.5:
            return f"{hz / 1e3:.3g}kHz"
        return f"{hz:.3g}Hz"


def build_accessor_from_core(core, num_channels: int = 16,
                             start_index: int = 0) -> SampleAccessor:
    """
    Tiện ích cho GUI: dựng SampleAccessor từ LogicAnalyzerCore.
    start_index > 0: chỉ lấy từ event thứ start_index (phân tích block capture
    cuối cùng, tránh seam giả khi Auto Run nối nhiều capture vào 1 timeline).
    """
    if start_index <= 0:
        return SampleAccessor(core.x_axis, core.channel_data, core.scaler_val, num_channels)
    return SampleAccessor(core.x_axis[start_index:],
                          [core.channel_data[c][start_index:] for c in range(num_channels)],
                          core.scaler_val, num_channels)
