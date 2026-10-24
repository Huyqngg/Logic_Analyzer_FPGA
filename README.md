# FPGA Logic Analyzer

**16 channels · up to 100 MSa/s · UART / I2C / SPI decoding · Python desktop GUI**

A logic analyzer built around an Intel Cyclone IV FPGA and a Python/Tkinter desktop application. The FPGA captures digital transitions into on-chip memory; the desktop application reconstructs waveforms, predicts channel roles, and overlays decoded protocol data. An ESP32 sketch supplies a reproducible set of test signals.

![Logic analyzer GUI showing 16 channels, detected roles, and decoded data](docs/images/gui-overview.jpg)

*GUI screenshot extracted from the project's development report. It shows one recorded development capture; channel activity and labels depend on the connected signals.*

## Highlights

- **Parallel acquisition:** 16 synchronized digital inputs, with a 50 MHz reference clock multiplied to 100 MHz by a PLL.
- **Event-based recording:** each record contains a 16-bit channel state and an 8-bit timestamp delta. Transitions and timestamp rollovers generate records.
- **Triggered capture:** selectable channel and rising/falling edge, with configurable pre-trigger memory and post-trigger acquisition.
- **Automatic channel identification:** heuristic detection of UART, I2C, SPI, PWM, clocks, counters, and idle channels, with confidence scores and channel-order invariance.
- **Protocol annotations:** UART bytes, SPI MOSI/MISO data, I2C addresses and data, and START/STOP/ACK/NACK markers.
- **Interactive inspection:** 16 waveform tracks, cursor-centered zoom, horizontal navigation, two-point time measurements, and manual role assignment through Decode Setup.
- **Hardware-independent testing:** synthetic captures exercise decoding, shuffled channel assignments, timestamp rollovers, truncated transactions, and cross-capture role accumulation.

The acquisition model is **buffered capture followed by readout**. Auto Run repeats this cycle; it does not provide uninterrupted 100 MSa/s streaming over UART.

## Architecture

Signal path: **DUT / ESP32 → input protection → FPGA capture buffer → USB-UART → Python GUI**.

![FPGA acquisition, trigger, memory, and UART architecture](docs/images/fpga-architecture.jpg)

The RTL synchronizes the probes, detects transitions, attaches timestamp deltas, and stores records in a circular buffer. After the trigger and post-capture phase, the PC requests buffer readout. The Python side reconstructs a shared time axis, classifies channel roles, and runs the decoders.

## Specifications

| Item | Configuration |
| --- | --- |
| FPGA | Cyclone IV E, EP4CE6E22C8; project board uses EP4CE6E22C8N |
| Input channels | 16, configured as 3.3 V LVTTL |
| Reference / internal clock | 50 MHz / 100 MHz |
| Sample rate | `100 MHz / (S + 1)`, where `S` is the scaler value |
| Maximum sample rate | 100 MSa/s; nominal 10 ns sample interval |
| Capture memory | 8,192 × 24-bit records, or 24 KiB |
| Record payload | 8-bit timestamp delta + 16-bit channel state |
| Host link | UART, 115200 baud, 8N1; RTL uses `CLKS_PER_BIT = 870` |
| Trigger | Selected channel, rising/falling edge; optional delay in RTL |
| Analysis | UART, I2C, SPI, PWM/clock measurements, counter recognition |

The GUI field labeled **Sample Rate (Divisor)** contains `S`, not `S + 1`:

| GUI value `S` | Sample rate | Time per tick |
| --- | --- | --- |
| 0 | 100 MSa/s | 10 ns |
| 1 | 50 MSa/s | 20 ns |
| 9 | 10 MSa/s | 100 ns |
| 99 | 1 MSa/s | 1 µs |

## Repository Contents

| Path | Purpose |
| --- | --- |
| [`src/`](src/) | Verilog acquisition, trigger, timestamp, buffer, UART, and PLL IP files |
| [`quartus_test/`](quartus_test/) | Quartus project, device selection, and FPGA pin assignments |
| [`firmware/`](firmware/) | Two original prebuilt SOF images for board bring-up |
| [`gui/main.py`](gui/main.py) | Desktop interface, serial capture, waveform rendering, and annotations |
| [`gui/sample_access.py`](gui/sample_access.py) | Time-axis and channel access helpers |
| [`gui/channel_predictor.py`](gui/channel_predictor.py) | Channel-role prediction and accumulation across captures |
| [`gui/decoders.py`](gui/decoders.py) | Protocol decoding and timing measurements |
| [`gui/test_decode_offline.py`](gui/test_decode_offline.py) | Hardware-independent regression suite |
| [`gui/fpga_link_check.py`](gui/fpga_link_check.py) | Serial-link and input-activity diagnostics |
| [`gui/capture_dump.py`](gui/capture_dump.py), [`gui/analyze_dump.py`](gui/analyze_dump.py) | Save hardware captures as JSON and inspect them offline |
| [`esp32_gen_input/`](esp32_gen_input/) | Arduino ESP32 test-signal generator |
| [`sim/top_testbench.v`](sim/top_testbench.v) | Original RTL unit/system simulation testbench |
| [`hardware/`](hardware/) | LTspice input-protection schematics and simulation runner |
| [`docs/images/`](docs/images/) | Selected GUI and architecture figures used in this README |

Reports, slides, presentation scripts, system-summary documents, generated simulation libraries, and Quartus build databases are omitted. The two named SOF images are retained deliberately as useful programming artifacts.

## Quick Start: Desktop Application

Use Python **3.10 or later** with Tkinter/Tcl-Tk support. The Python regression suite was run with Python 3.12 while preparing this repository.

From the repository root, on Windows:

```powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe gui/main.py
```

On Linux/macOS, use the interpreter inside `.venv/bin/`:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python gui/main.py
```

Tkinter is supplied by the Python installation or operating system, rather than pip. Verify it with `python -m tkinter` if the GUI cannot open.

### Capture Workflow

1. Program the FPGA and connect a 3.3 V USB-UART adapter: adapter TX → FPGA `i_rx`, adapter RX ← FPGA `o_tx`, with common ground.
2. Connect the DUT's signals to the probe inputs. Load the ESP32 sketch if using the included demo signals.
3. Start the application, select the FPGA's USB-UART port, and connect. The ESP32's debug serial port is a separate connection.
4. Start with **trigger channel 1**, **Falling**, **pre-capture 1024**, and **scaler 1** for the supplied demo burst pattern.
5. Enable Auto Decode and, optionally, Auto Run, then choose Start Capture.
6. Zoom or pan through the traces and click two points to measure a time interval. Use Decode Setup to correct ambiguous channel roles or bus groupings.

## FPGA Build and Programming

The supplied project was created with **Quartus Prime Lite 22.1**, targeting **EP4CE6E22C8**.

Open [`quartus_test/logic_analyzer_top.qpf`](quartus_test/logic_analyzer_top.qpf) in Quartus, check the board and pin assignments, and compile. With the Quartus command-line tools available:

```bash
cd quartus_test
quartus_sh --flow compile logic_analyzer_top
```

Program the resulting SOF file using Quartus Programmer and the board's programming adapter. The original images are also included:

| Image | Intended wiring, as documented in the original project |
| --- | --- |
| [`firmware/la_khong_dao.sof`](firmware/la_khong_dao.sof) | Direct 3.3 V probes; `INVERT_PROBE = 0` |
| [`firmware/la_co_dao.sof`](firmware/la_co_dao.sof) | Probes through the 74LVC14A inverting protection stage; `INVERT_PROBE = 1` |

The source defaults to `INVERT_PROBE = 0`. Set this top-level parameter to match the input stage when rebuilding. The supplied images are original build artifacts; rebuild after changing RTL, device settings, or board wiring.

### Board Connections and Demo Signals

The `.qsf` file is the authoritative source for this board's FPGA pin mapping:

| Signal | FPGA pin |
| --- | --- |
| 50 MHz clock, `i_clk` | PIN_24 |
| Active-low reset, `i_rstn` | PIN_88 |
| UART receive, `i_rx` | PIN_110 |
| UART transmit, `o_tx` | PIN_111 |

The ESP32 sketch uses the Arduino ESP32 **3.x LEDC API** and defines the following demo wiring:

| Channel | FPGA probe pin | ESP32 GPIO | Demo role |
| --- | --- | --- | --- |
| CH-1 | PIN_144 | 17 | UART-A TX, known byte pattern |
| CH-2 | PIN_143 | 21 | UART-A device response, `Hi!` |
| CH-3 | PIN_141 | 22 | I2C-A SDA |
| CH-4 | PIN_138 | 5 | I2C-A SCL |
| CH-5 | PIN_137 | 18 | SPI SCK |
| CH-6 | PIN_135 | 23 | SPI MOSI |
| CH-7 | PIN_133 | 16 | SPI MISO |
| CH-8 | PIN_132 | 19 | SPI CS |
| CH-9 | PIN_129 | 2 | Bit-banged UART-B |
| CH-10 | PIN_128 | 4 | I2C-B SDA, address `0x50` / NACK case |
| CH-11 | PIN_127 | 12 | I2C-B SCL |
| CH-12 | PIN_126 | 13 | PWM, 1 kHz / approximately 75% |
| CH-13 | PIN_125 | 14 | PWM, 10 kHz / approximately 25% |
| CH-14 | PIN_124 | 15 | Clock, 1 MHz |
| CH-15 | PIN_121 | 25 | Clock, 500 kHz |
| CH-16 | PIN_120 | 26 | Clock, 100 kHz |

Follow the sketch's self-slave connections for the I2C and SPI demo responses. Use 3.3 V logic at the FPGA pins; voltage tolerance provided by an external protection stage must not be assumed for bare FPGA inputs. The LTspice schematics model the optional protection circuit, whose inversion is compensated by `INVERT_PROBE`.

## Tests and Diagnostics

### Offline Decoder Regression

Run from the repository root; no FPGA or serial device is required:

```bash
python gui/test_decode_offline.py
```

The supplied suite passed during repository preparation:

- **144/144 role assignments** across the baseline and eight shuffled channel layouts.
- Correct identification with injected timestamp rollovers.
- I2C repeated START, clock stretching, and NACK-only transactions.
- UART parity/stop-bit checks and measured-baud decoding.
- SPI MOSI/MISO recognition, truncated captures, false-positive rejection, and cross-capture role merging.

The existing RTL testbench is retained for Questa/ModelSim. It needs Intel's PLL simulation library (`altera_mf_ver`) when using `pll_sys.v`; do not compile `pll_sys_bb.v` as a second implementation of the same module. RTL simulation, Quartus compilation, LTspice simulations, and physical hardware were not rerun during repository preparation.

### Link Check and Capture Dumps

Replace `COM5` with the connected FPGA adapter, or use a port such as `/dev/ttyUSB0`:

```bash
python gui/fpga_link_check.py COM5 --trig 16
python gui/capture_dump.py --port COM5 --trig 1 --edge fall --scaler 1 --precap 1024 --out gui/dumps/capture.json
python gui/analyze_dump.py gui/dumps/capture.json
```

Use the `.qsf` pin mapping above when wiring hardware; some original diagnostic messages still refer to an earlier pin layout.

The optional protection-circuit runner is `hardware/sim/run_all.py`. It currently expects LTspice installed through Wine; its `LTSPICE` path must match the local installation. Generated plots, raw simulation data, and results remain ignored.

## Data Format

Each captured record is sent as four bytes: **`[header, delta_ticks, channels_low, channels_high]`**. Channel 1 is bit 0 of the reconstructed 16-bit channel word.

| Header | Meaning |
| --- | --- |
| `0xA1` | Pre-trigger record |
| `0xA3` | Post-trigger record |
| `0xA7` | Trigger notification |
| `0xAF` | Buffer-full notification |

`0xA7` and `0xAF` are capture-phase notifications, separate from the four-byte data records. The PC sends two-byte `[command, value]` messages. Scaler (`0xFA`) and pre-capture depth (`0xFE`) each require two such messages, high byte first and then low byte. Other commands select the trigger channel (`0xFB`), edge (`0xFC`), enable (`0xFD`), readout (`0xF9`), stop (`0xFF`), and optional trigger delay (`0xF7` / `0xF8`).

Time is reconstructed using `elapsed_ns = cumulative_ticks × (S + 1) × 10`.

## Performance and Limits

- A full buffer contains 32,768 UART data bytes. At 115200 baud / 8N1, the theoretical readout time is approximately **2.84 seconds**, before software overhead.
- Capture duration depends on transition density and the records inserted when the 8-bit timestamp rolls over.
- Auto Run joins capture blocks for display. The dead time between blocks is not a continuously measured waveform interval.
- Fast clocks and slow transactions may need separate captures with different scaler values. Ambiguous patterns can require manual channel assignments.
- The original Quartus build reports an internal-clock Fmax of **131.13 MHz**, **514 / 6,272 logic elements**, and **196,608 / 276,480 memory bits**. These are historical build results; the original timing report also lists unconstrained external I/O paths. Review timing constraints when adapting the design to another board.

## Team and Provenance

Developed for the Embedded Systems and Embedded Interface Design course at Hanoi University of Science and Technology.

| Team member | Primary contribution |
| --- | --- |
| Nguyễn Quốc Huy | PC channel-role prediction and protocol decoders |
| Trần Độ | Desktop GUI, serial interface, and waveform display |
| Tạ Văn Việt | Project coordination, FPGA, PCB, and UART integration |
| Trần Phi Anh Nhật | ESP32 test signals, component selection, and protection schematic |

The GUI and FPGA architecture images were extracted from Figures 9 and 6 of the team's original report. The full report is intentionally omitted from this repository. Existing Intel notices in the generated PLL IP and Quartus files are retained; no additional repository-wide license is assigned here.
