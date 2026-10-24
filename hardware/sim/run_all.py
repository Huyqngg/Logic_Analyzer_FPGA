#!/usr/bin/env python3
"""
run_all.py — kiểm thử tự động mạch bảo vệ IO bằng LTspice (batch qua Wine).

Mỗi ca thử: sinh netlist -> chạy LTspice -b -ascii -> parse .raw -> tính chỉ số
-> chấm PASS/FAIL theo tiêu chí định lượng -> vẽ PNG. Kết quả gộp vào
out/results.json để make_report.py nhúng vào PDF.

Chạy:  cd hardware/sim && python3 run_all.py
"""

import json
import os
import subprocess
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LTSPICE = os.path.expanduser(
    "~/.wine/drive_c/Program Files/ADI/LTspice/LTspice.exe")
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
os.makedirs(OUT, exist_ok=True)

# Mạch 1 kênh (đúng netlist đã verify từ bao_ve_io_1kenh.asc)
# A1 = 74LVC14A Schmitt INVERTER: output ở node thứ 6 (ngõ đảo) → V(fpga) = NOT V(in).
# FPGA đảo lại bằng INVERT_PROBE=1 trong logic_analyzer_top.v.
CIRCUIT = """* bao ve io - {name}
V1 in 0 {stim}
Rin1 prot in 220
D1 0 prot DTVS
R1 prot 0 1Meg
C1 prot 0 {c1}
Cin prot 0 5p
A1 prot 0 0 0 0 bufo 0 0 SCHMITT Vt={vt} Vh=0.35 Vhigh=3.3 Vlow=0 Trise=3n Tfall=3n
Rdrv drv bufo 18
Rout1 fpga drv 33
Cload fpga 0 6p
.model DTVS D(BV=6.1 IBV=1m Cjo=0.5p Is=1e-12 Rs=0.5)
.tran 0 {tstop} 0 {maxstep}
.end
"""


def run_spice(name, stim, tstop, maxstep, vt="1.25", c1="2.2p"):
    net = os.path.join(OUT, f"{name}.net")
    with open(net, "w") as f:
        f.write(CIRCUIT.format(name=name, stim=stim, tstop=tstop,
                               maxstep=maxstep, vt=vt, c1=c1))
    # LƯU Ý: phải chạy với cwd=OUT + tên file TƯƠNG ĐỐI — LTspice qua Wine coi
    # đường dẫn Unix tuyệt đối (/home/...) là cờ tùy chọn kiểu Windows.
    subprocess.run(["wine", LTSPICE, "-b", "-ascii", f"{name}.net"],
                   cwd=OUT, capture_output=True, timeout=300)
    return parse_raw(os.path.join(OUT, f"{name}.raw"))


def parse_raw(path):
    """Parse file .raw ASCII của LTspice -> dict tên_biến -> np.array."""
    with open(path, errors="replace") as f:
        lines = f.read().splitlines()
    names, n_vars, i = [], 0, 0
    while i < len(lines):
        ln = lines[i]
        if ln.startswith("No. Variables:"):
            n_vars = int(ln.split(":")[1])
        elif ln.startswith("Variables:"):
            for k in range(n_vars):
                names.append(lines[i + 1 + k].split("\t")[2])
            i += n_vars
        elif ln.startswith("Values:"):
            i += 1
            break
        i += 1
    data = [[] for _ in range(n_vars)]
    v = 0
    for ln in lines[i:]:
        ln = ln.strip()
        if not ln:
            continue
        parts = ln.split("\t")
        val = parts[-1]
        data[v].append(float(val))
        v = (v + 1) % n_vars
    return {nm: np.array(col) for nm, col in zip(names, data)}


def crossings(t, v, level, rising=True):
    """Thời điểm v cắt `level` (nội suy tuyến tính)."""
    v0, v1 = v[:-1], v[1:]
    if rising:
        idx = np.where((v0 < level) & (v1 >= level))[0]
    else:
        idx = np.where((v0 > level) & (v1 <= level))[0]
    out = []
    for k in idx:
        f = (level - v0[k]) / (v1[k] - v0[k])
        out.append(t[k] + f * (t[k + 1] - t[k]))
    return np.array(out)


def plot(name, d, sigs, title, xlim=None, xscale=1e6, xlabel="t (µs)"):
    fig, ax = plt.subplots(figsize=(9, 4.2))
    for s, lb in sigs:
        if s in d:
            ax.plot(d["time"] * xscale, d[s], label=lb, lw=1.2)
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("V")
    if xlim:
        ax.set_xlim(xlim)
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    p = os.path.join(OUT, f"{name}.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


R = {}   # kết quả


def case1_dc5v():
    d = run_spice("ca1_dc5v", "PWL(0 0 1u 0 1.01u 5 10m 5)",
                  "10m", "2u")
    m = d["time"] > 2e-3        # xét sau khi xác lập
    vprot = float(d["V(prot)"][m].max())
    vfpga_end = float(d["V(fpga)"][-1])
    isrc = float(np.abs(d["I(V1)"][m]).max())
    # LVC14 đảo: input mức cao ổn định -> chân FPGA = 0V (mức hợp lệ; sau INVERT_PROBE đọc = 1)
    ok = vprot <= 5.5 and vfpga_end <= 0.3 and isrc < 1e-3
    R["ca1"] = dict(
        ten="Cắm nhầm 5V DC (10ms)",
        so_lieu=f"V(prot)={vprot:.2f}V (giới hạn 5.5V) | V(fpga)={vfpga_end:.2f}V "
                f"(mức 0 hợp lệ — sau INVERT_PROBE FPGA đọc = 1) | I nguồn={isrc*1e6:.1f}µA (<1mA)",
        dat=ok,
        png=plot("ca1_dc5v", d, [("V(in)", "V vào"), ("V(prot)", "V tại input buffer"),
                                 ("V(fpga)", "V chân FPGA")],
                 "Ca 1: cắm nhầm 5V DC — input LVC chịu 5.5V nên KHÔNG hỏng",
                 xscale=1e3, xlabel="t (ms)"))


def case2_surge12():
    d = run_spice("ca2_surge12", "PWL(0 0 10u 0 10.1u 12 1.01m 12 1.011m 0 1.2m 0)",
                  "1.2m", "200n")
    m = (d["time"] > 20e-6) & (d["time"] < 1e-3)
    vprot = float(d["V(prot)"][m].max())
    i_tvs = float(np.abs(d["I(D1)"][m]).max())
    p_rin = float((((d["V(in)"] - d["V(prot)"]) ** 2) / 220.0)[m].max())
    ok = vprot <= 6.8 and i_tvs < 0.2
    R["ca2"] = dict(
        ten="Surge 12V / 1ms",
        so_lieu=f"V(prot) kẹp còn {vprot:.2f}V (giới hạn 6.8V) | I(TVS)={i_tvs*1e3:.1f}mA "
                f"| P(Rin 220Ω) đỉnh={p_rin*1e3:.0f}mW (điện trở 1/4W chịu tốt trong 1ms)",
        dat=ok,
        png=plot("ca2_surge12", d, [("V(in)", "V vào 12V"), ("V(prot)", "V tại input buffer (kẹp)"),
                                    ("V(fpga)", "V chân FPGA")],
                 "Ca 2: surge 12V — TVS kẹp, chân FPGA không bao giờ thấy quá áp",
                 xscale=1e3, xlabel="t (ms)"))


def case3_neg5v():
    d = run_spice("ca3_neg5v", "PWL(0 0 1u 0 1.01u -5 1m -5)", "1m", "1u")
    m = d["time"] > 10e-6
    vprot = float(d["V(prot)"][m].min())
    isrc = float(np.abs(d["I(V1)"][m]).max())
    ok = vprot >= -1.0
    R["ca3"] = dict(
        ten="Điện áp âm −5V",
        so_lieu=f"V(prot) kẹp ở {vprot:.2f}V (giới hạn −1.0V) | dòng qua Rin={isrc*1e3:.1f}mA",
        dat=ok,
        png=plot("ca3_neg5v", d, [("V(in)", "V vào −5V"), ("V(prot)", "V tại input buffer (kẹp diode)"),
                                  ("V(fpga)", "V chân FPGA")],
                 "Ca 3: −5V — diode TVS dẫn thuận, input buffer chỉ thấy ~−0.8V",
                 xscale=1e3, xlabel="t (ms)"))


def _clock_metrics(d):
    t = d["time"]
    rin = crossings(t, d["V(in)"], 1.65, True)
    rout = crossings(t, d["V(fpga)"], 1.65, True)
    fin = crossings(t, d["V(in)"], 1.65, False)
    fout = crossings(t, d["V(fpga)"], 1.65, False)
    # LVC14 đảo mức: cạnh LÊN ở input ứng với cạnh XUỐNG ở FPGA
    # trễ lan truyền: ghép cạnh ra (xuống) gần nhất SAU mỗi cạnh vào (lên)
    delays = []
    for te in rin:
        nxt = fout[fout > te]
        if len(nxt):
            delays.append(nxt[0] - te)
    delay = float(np.median(delays)) if delays else float("nan")
    # duty tại chân FPGA vs nguồn
    def duty(rr, ff):
        if len(rr) < 2 or len(ff) < 1:
            return float("nan")
        period = np.median(np.diff(rr))
        highs = [f - r for r in rr for f in ff[(ff > r) & (ff < r + period)][:1]]
        return float(np.median(highs) / period)
    d_in, d_out = duty(rin, fin), duty(rout, fout)
    over = float(d["V(fpga)"].max())
    return delay, d_in, d_out, over


def case4_clk1mhz():
    d = run_spice("ca4_clk1mhz", "PULSE(0 3.3 100n 5n 5n 495n 1000n)", "5u", "1n")
    delay, d_in, d_out, over = _clock_metrics(d)
    # LVC14 đảo: duty ở FPGA = 1 - duty vào; so sánh sau khi bù đảo
    duty_err = abs((1.0 - d_out) - d_in) * 100
    ok = delay <= 20e-9 and duty_err <= 2.0 and over <= 3.3 * 1.10
    R["ca4"] = dict(
        ten="Clock 1MHz 3.3V (tín hiệu nhanh nhất của hệ)",
        so_lieu=f"Trễ lan truyền={delay*1e9:.1f}ns (giới hạn 20ns) | méo duty sau bù đảo="
                f"{duty_err:.2f}% (giới hạn 2%) | đỉnh V(fpga)={over:.2f}V (giới hạn 3.63V)",
        dat=ok,
        png=plot("ca4_clk1mhz", d, [("V(in)", "V vào"), ("V(prot)", "V input buffer"),
                                    ("V(fpga)", "V chân FPGA (đảo mức)")],
                 "Ca 4: clock 1MHz qua mạch bảo vệ — Schmitt tái tạo sạch, FPGA nhận sóng đảo",
                 xlim=(0, 3)))


def case5_uart_edge():
    # start-bit UART 115200: xung thấp 8.68µs
    d = run_spice("ca5_uart", "PULSE(3.3 0 1u 5n 5n 8.68u 40u)", "15u", "1n")
    t = d["time"]
    fin = crossings(t, d["V(in)"], 1.65, False)
    rin = crossings(t, d["V(in)"], 1.65, True)
    fout = crossings(t, d["V(fpga)"], 1.65, False)
    rout = crossings(t, d["V(fpga)"], 1.65, True)
    w_in = float(rin[0] - fin[0])
    # LVC14 đảo: xung THẤP 8.68µs ở input thành xung CAO cùng độ rộng ở FPGA
    w_out = float(fout[0] - rout[0])
    err = abs(w_out - w_in) / w_in * 100
    ok = err < 1.0
    R["ca5"] = dict(
        ten="Cạnh UART 115200 (bit 8.68µs)",
        so_lieu=f"độ rộng bit vào={w_in*1e6:.3f}µs, ra (xung đảo)={w_out*1e6:.3f}µs "
                f"→ sai lệch {err:.3f}% (giới hạn 1%) — decode không bị ảnh hưởng",
        dat=ok,
        png=plot("ca5_uart", d, [("V(in)", "V vào (start bit)"), ("V(fpga)", "V chân FPGA (đảo mức)")],
                 "Ca 5: bit UART 8.68µs — độ rộng xung giữ nguyên qua mạch (dạng đảo)",
                 xlim=(0, 12)))


def case6_amplitudes():
    rows = []
    all_ok = True
    for vamp in (1.2, 1.8, 2.5, 3.3, 5.0):
        for vt, corner in (("1.25", "điển hình VT+=1.6V"), ("1.65", "xấu nhất VT+=2.0V")):
            nm = f"ca6_a{str(vamp).replace('.','p')}_vt{vt.replace('.','p')}"
            d = run_spice(nm, f"PULSE(0 {vamp} 100n 5n 5n 495n 1000n)", "5u", "2n", vt=vt)
            toggles = d["V(fpga)"].max() > 3.0 and d["V(fpga)"].min() < 0.3
            expect = vamp >= (float(vt) + 0.35)  # phải vượt VT+
            rows.append((vamp, corner, "CÓ" if toggles else "KHÔNG",
                         "đúng kỳ vọng" if toggles == expect else "LỆCH KỲ VỌNG"))
            if toggles != expect:
                all_ok = False
    # bảng: logic 3.3/5V phải đọc được ở MỌI corner; 2.5V đọc được ở corner điển hình
    txt = " | ".join(f"{v}V/{c.split()[0]}:{t}" for v, c, t, _ in rows)
    R["ca6"] = dict(
        ten="Quét mức logic 1.2/1.8/2.5/3.3/5V × 2 corner ngưỡng Schmitt",
        so_lieu="; ".join(f"{v}V ({c}) → FPGA {'nhận' if t=='CÓ' else 'KHÔNG nhận'}"
                          for v, c, t, _ in rows)
                + ". Mức không qua ngưỡng: chân FPGA đứng yên 3.3V, sau INVERT_PROBE đọc 0 cố định (idle) — an toàn.",
        dat=all_ok,
        bang=rows,
        png="")


def case7_effect_c1():
    d_with = run_spice("ca7_coC", "PULSE(0 3.3 100n 5n 5n 495n 1000n)", "5u", "1n")
    d_no = run_spice("ca7_khongC", "PULSE(0 3.3 100n 5n 5n 495n 1000n)", "5u", "1n",
                     c1="0.001p")
    def rise(d):
        t = d["time"]
        t10 = crossings(t, d["V(prot)"], 0.33, True)
        t90 = crossings(t, d["V(prot)"], 2.97, True)
        return float(t90[0] - t10[0]) if len(t10) and len(t90) else float("nan")
    r_with, r_no = rise(d_with), rise(d_no)
    delay_w, *_ = _clock_metrics(d_with)
    delay_n, *_ = _clock_metrics(d_no)
    ok = abs(delay_w - delay_n) < 5e-9
    fig, ax = plt.subplots(figsize=(9, 4.2))
    ax.plot(d_with["time"] * 1e6, d_with["V(prot)"], label="có C=2.2pF", lw=1.2)
    ax.plot(d_no["time"] * 1e6, d_no["V(prot)"], label="không C", lw=1.2, ls="--")
    ax.set_title("Ca 7: so sánh có/không tụ 2.2pF tại input buffer")
    ax.set_xlabel("t (µs)"); ax.set_ylabel("V")
    ax.set_xlim(0.05, 0.25); ax.grid(alpha=0.3); ax.legend(fontsize=9)
    fig.tight_layout()
    p7 = os.path.join(OUT, "ca7_soC.png")
    fig.savefig(p7, dpi=130); plt.close(fig)
    R["ca7"] = dict(
        ten="Ảnh hưởng của C 2.2pF (nhiễu/méo?)",
        so_lieu=f"rise time tại input buffer: có C={r_with*1e9:.2f}ns, bỏ C={r_no*1e9:.2f}ns; "
                f"trễ tổng: {delay_w*1e9:.1f}ns vs {delay_n*1e9:.1f}ns → chênh không đáng kể; "
                "C 2.2pF chỉ lọc nhiễu cao tần >100MHz, KHÔNG làm méo tín hiệu đo",
        dat=ok,
        png=p7)


def main():
    for fn in (case1_dc5v, case2_surge12, case3_neg5v, case4_clk1mhz,
               case5_uart_edge, case6_amplitudes, case7_effect_c1):
        fn()
        k = sorted(R)[-1]
    print(f"\n{'='*70}")
    n_ok = 0
    for k in sorted(R):
        r = R[k]
        tag = "PASS" if r["dat"] else "FAIL"
        n_ok += r["dat"]
        print(f"[{tag}] {r['ten']}\n       {r['so_lieu']}")
    print(f"{'='*70}\nKẾT QUẢ: {n_ok}/{len(R)} ca ĐẠT")
    with open(os.path.join(OUT, "results.json"), "w") as f:
        json.dump(R, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
