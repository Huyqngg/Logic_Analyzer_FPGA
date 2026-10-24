#!/usr/bin/env python3
"""
analyze_dump.py — mổ xẻ 1 dump capture THẬT: chạy predictor + decoder offline,
in chi tiết vì sao từng kênh được/không được nhận vai trò.

Chạy:  python3 analyze_dump.py dumps/*.json
"""

import sys

from sample_access import SampleAccessor
from capture_dump import load_dump
from channel_predictor import (predict_roles, _i2c_score, _sda_scl_edge_split,
                               _best_spi_triple, _continuity)
from decoders import (decode_all, decode_i2c, measure_uart_bit_ticks,
                      estimate_uart_bit_ticks, uart_frame_quality)

# Sơ đồ thật của ESP32 v2 (0-based) — chỉ để ĐỐI CHIẾU khi in
EXPECTED = ["UART-A TX", "UART-A RX", "I2C-A SDA", "I2C-A SCL",
            "SPI SCK", "SPI MOSI", "SPI MISO", "SPI CS",
            "UART-B", "I2C-B SDA", "I2C-B SCL",
            "PWM 1kHz75", "PWM 10kHz25", "CLK 1MHz", "CLK 500kHz", "CLK 100kHz"]


def analyze(path):
    x_axis, channel_data, scaler_val = load_dump(path)
    sa = SampleAccessor(x_axis, channel_data, scaler_val, 16)
    print(f"\n{'='*72}\n{path}: {sa.n} event, {sa.duration_ticks} tick, "
          f"scaler_val={scaler_val} ({sa.sample_rate_hz/1e6:.0f}MHz)\n{'='*72}")

    roles = predict_roles(sa, uart_baud=115200)
    print(f"{'CH':4s} {'thật là':12s} {'dự đoán':10s} {'label':22s} "
          f"{'conf':5s} {'trans':6s} {'gated':6s} {'cover':6s}")
    for r in roles:
        print(f"CH{r.ch+1:<3d} {EXPECTED[r.ch]:12s} {r.role:10s} {r.label:22s} "
              f"{r.confidence:.2f}  {sa.num_transitions(r.ch):5d}  "
              f"{str(sa.is_gated(r.ch)):6s} {sa.activity_coverage(r.ch):.2f}")

    # ---- I2C: soi trực tiếp 2 cặp thật --------------------------------------
    for scl, sda, tag in [(3, 2, "I2C-A"), (10, 9, "I2C-B")]:
        n_hi, n_lo = _sda_scl_edge_split(sa, scl, sda)
        anns = decode_i2c(sa, scl, sda)
        kinds = {}
        for a in anns:
            kinds[a.kind] = kinds.get(a.kind, 0) + 1
        score, ev = _i2c_score(sa, scl, sda)
        print(f"\n[{tag}] scl=CH{scl+1} sda=CH{sda+1}: idle=({sa.idle_level(scl)},"
              f"{sa.idle_level(sda)}) trans_scl={sa.num_transitions(scl)} "
              f"edge_split(hi/lo)={n_hi}/{n_lo} anns={kinds} score={score:.1f} ev={ev}")
        bytes_txt = [a.text for a in anns if a.kind == 'byte'][:8]
        if bytes_txt:
            print(f"        bytes: {bytes_txt}")

    # ---- UART: soi 3 kênh thật ----------------------------------------------
    for ch, tag in [(0, "UART-A TX"), (1, "UART-A RX"), (8, "UART-B")]:
        meas = measure_uart_bit_ticks(sa, ch)
        est = estimate_uart_bit_ticks(sa, ch, 115200)
        frames, expl, bit = uart_frame_quality(sa, ch, baud=115200)
        vals = [f.text for f in frames[:8]]
        baud_m = sa.sample_rate_hz / est if est else 0
        print(f"[{tag}] CH{ch+1}: đo={meas:.0f} tick, dùng={est:.0f} tick "
              f"(~{baud_m/1e3:.1f}kbd), frames={len(frames)} expl={expl:.2f} {vals}")

    # ---- SPI ------------------------------------------------------------------
    spi = _best_spi_triple(sa, set(range(16)))
    print(f"[SPI] best_triple={spi}")

    # ---- PWM/CLK ----------------------------------------------------------------
    for ch in (11, 12, 13, 14, 15):
        info = _continuity(sa, ch)
        if info:
            print(f"[CONT] CH{ch+1} ({EXPECTED[ch]}): f={info['freq']:.0f}Hz "
                  f"duty={info['duty']*100:.0f}% periods={info['periods']:.1f} ")
        else:
            print(f"[CONT] CH{ch+1} ({EXPECTED[ch]}): KHÔNG liên tục "
                  f"(cv={_period_cv_dbg(sa, ch):.2f}, gated={sa.is_gated(ch)})")


def _period_cv_dbg(sa, ch):
    r = sa.edges(ch, "rising")
    if len(r) < 3:
        return -1.0
    diffs = [r[i + 1] - r[i] for i in range(len(r) - 1)]
    m = sum(diffs) / len(diffs)
    if m == 0:
        return -1.0
    var = sum((d - m) ** 2 for d in diffs) / len(diffs)
    return (var ** 0.5) / m


if __name__ == "__main__":
    for p in sys.argv[1:]:
        analyze(p)
