import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import numpy as np

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.widgets import Slider, Button

# --- Lớp DECODE / PREDICT (P3) ---
from sample_access import build_accessor_from_core
from channel_predictor import predict_roles, ChannelRole, RoleAccumulator
from decoders import decode_all

UART_BAUD = 115200

# Dải hiển thị dọc mỗi kênh: chừa "băng nhãn" phía trên (1.2 -> 1.6) cho overlay byte/marker
YLIM_BOTTOM = -0.2
YLIM_TOP = 1.6
OVERLAY_Y = 1.35

# Danh sách vai trò cho dialog chỉnh tay + nhãn mặc định
ROLE_CHOICES = [
    "UART", "SPI_SCK", "SPI_MOSI", "SPI_MISO", "SPI_CS", "I2C_SCL", "I2C_SDA",
    "CLOCK", "PWM", "COUNTER_BIT", "IDLE", "UNKNOWN",
]
ROLE_DEFAULT_LABEL = {
    "UART": "uart-data", "SPI_SCK": "spi-sck", "SPI_MOSI": "spi-mosi",
    "SPI_MISO": "spi-miso", "SPI_CS": "spi-cs", "I2C_SCL": "i2c-scl",
    "I2C_SDA": "i2c-sda", "CLOCK": "clk", "PWM": "pwm", "COUNTER_BIT": "counter",
    "IDLE": "idle", "UNKNOWN": "unknown",
}
# Màu marker theo loại annotation
MARKER_STYLE = {
    "start": ("#00aa00", "-"), "stop": ("#cc0000", "-"),
    "ack": ("#0077cc", ":"), "nack": ("#ee8800", ":"),
}

# =========================================================
# FPGA PROTOCOL HEADERS
# =========================================================
CMD_SET_SCALER       = 0xFA
CMD_SET_PRECAP_DEPTH = 0xFE
CMD_SET_CHANNEL      = 0xFB
CMD_SET_TRIG_TYPE    = 0xFC
CMD_SET_DELAY_EN     = 0xF7
CMD_SET_DELAY_VAL    = 0xF8
CMD_SET_ENABLE       = 0xFD
CMD_SET_STOP         = 0xFF
CMD_SET_START_READ   = 0xF9

HDR_PRE_BUFF   = 0xA1
HDR_POST_BUFF  = 0xA3
HDR_TRIGGERED  = 0xA7
HDR_DONE       = 0xAF

class LogicAnalyzerCore:
    def __init__(self):
        self.port = ""
        self.baud = 115200
        self.num_channels = 16
        
        self.channel_data = [[] for _ in range(self.num_channels)]
        self.x_axis = []
        self.trigger_times = [] # Lưu danh sách tất cả các điểm sập bẫy trong Pipeline
        self.last_timestamp = 0 # Lưu thời gian cộng dồn của gói tin cuối cùng
        self.scaler_val = 1 
        self.ser = None

    def get_ports(self):
        return [port.device for port in serial.tools.list_ports.comports(include_links=False)]

    def send_config(self, channel, trig_type, scaler, precap):
        if not self.ser or not self.ser.is_open: return
        self.scaler_val = scaler + 1 
        try:
            print("\n" + "="*50, flush=True)
            print(f"[PC -> FPGA] GỬI CẤU HÌNH: CH={channel}, Edge={'Rise' if trig_type else 'Fall'}, Scaler={scaler}, Precap={precap}", flush=True)
            self.ser.reset_input_buffer()
            self.ser.write(bytes([CMD_SET_SCALER, (scaler >> 8) & 0xFF]))
            self.ser.write(bytes([CMD_SET_SCALER, scaler & 0xFF]))
            self.ser.write(bytes([CMD_SET_PRECAP_DEPTH, (precap >> 8) & 0xFF]))
            self.ser.write(bytes([CMD_SET_PRECAP_DEPTH, precap & 0xFF]))
            self.ser.write(bytes([CMD_SET_CHANNEL, channel]))
            self.ser.write(bytes([CMD_SET_TRIG_TYPE, trig_type]))
            
            self.ser.write(bytes([CMD_SET_DELAY_EN, 0x00]))
            self.ser.write(bytes([CMD_SET_DELAY_VAL, 0x00])) 
            
            self.ser.write(bytes([CMD_SET_ENABLE, 0x00]))
            self.ser.write(bytes([CMD_SET_START_READ, 0x00]))
            print("="*50 + "\n", flush=True)
        except Exception as e:
            print(f"[LỖI] Không thể gửi Config: {e}", flush=True)

class EnxorMainApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Logic Analyzer Workspace")
        self.geometry("1200x850")
        
        self.core = LogicAnalyzerCore()
        self.capture_thread = None
        self.stop_requested = False
        
        # Đo khoảng thời gian
        self.is_first_coord = True
        self.x1 = 0.0
        self.x2 = 0.0
        
        # Biến trạng thái hiển thị
        self.zoom_left = 0
        self.zoom_right = 0
        self.total_time_units = 0
        self.is_first_plot = True
        self.drawn_triggers = 0
        
        self.fig = None
        self.axs = None
        self.spos = None
        self.btn = None
        self.canvas = None
        self.step_lines = []

        # --- DECODE / PREDICT state ---
        self.channel_roles = None      # list[ChannelRole] (16) - dự đoán hoặc chỉnh tay
        self.overlay_artists = []      # artist overlay (text/vline) để xóa & vẽ lại
        self.max_overlay = 500         # giới hạn số annotation vẽ (tránh lag)
        # Niềm tin vai trò gộp qua NHIỀU capture: cửa sổ ~2.66ms < 1 vòng burst
        # ESP32 -> mỗi capture chỉ thấy 1 phần giao thức; Auto Run hội tụ dần.
        self.role_accum = RoleAccumulator(self.core.num_channels)
        # Index bắt đầu của từng block capture trong x_axis (Auto Run nối đuôi
        # các capture) -> predict/decode chỉ chạy trên block CUỐI, tránh seam giả.
        self.capture_block_starts = []
        
        self.status_var = tk.StringVar(value="READY")
        self.time_var = tk.StringVar(value="---")
        self.time_diff_var = tk.StringVar(value="0.00 ns")
        self.selected_port = tk.StringVar()
        
        self.setup_menu()
        self.setup_header()
        self.setup_waveform_window()
        self.setup_footer()

    def setup_menu(self):
        menubar = tk.Menu(self)
        self.capture_menu = tk.Menu(menubar, tearoff=0)
        self.capture_menu.add_command(label="Start Capture", command=self.start_capture)
        self.capture_menu.add_command(label="Stop Capture", command=self.stop_capture)
        menubar.add_cascade(label="Capture", menu=self.capture_menu)
        
        self.settings_menu = tk.Menu(menubar, tearoff=0)
        self.ports_menu = tk.Menu(self.settings_menu, tearoff=0, postcommand=self.refresh_ports)
        self.settings_menu.add_cascade(label="Serial Ports", menu=self.ports_menu)
        menubar.add_cascade(label="Settings", menu=self.settings_menu)

        # --- Menu DECODE ---
        self.decode_menu = tk.Menu(menubar, tearoff=0)
        self.decode_menu.add_command(label="Auto Predict + Decode", command=self.run_predict_and_decode)
        self.decode_menu.add_command(label="Re-decode (giữ vai trò)", command=lambda: self.run_decode(predict=False))
        self.decode_menu.add_separator()
        self.decode_menu.add_command(label="Decode Setup (chỉnh tay)...", command=self.open_decode_setup)
        self.decode_menu.add_command(label="Clear Overlay", command=self.clear_overlay_and_labels)
        menubar.add_cascade(label="Decode", menu=self.decode_menu)
        self.config(menu=menubar)

    def refresh_ports(self):
        self.ports_menu.delete(0, 'end')
        for port in self.core.get_ports():
            self.ports_menu.add_radiobutton(label=port, variable=self.selected_port, value=port, command=self.connect_port)

    def connect_port(self):
        port = self.selected_port.get()
        try:
            if self.core.ser: self.core.ser.close()
            self.core.ser = serial.Serial(port, self.core.baud, timeout=0.5)
            self.status_var.set("READY")
            self.core.port = port
            print(f"[HỆ THỐNG] Đã mở cổng {port} thành công.", flush=True)
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể mở {port}\n{e}")

    def setup_header(self):
        header = tk.Frame(self)
        header.pack(fill=tk.X, side=tk.TOP, padx=5, pady=5)
        
        tk.Label(header, text="Trigger Channel").pack(side=tk.LEFT, padx=5)
        self.cb_channel = ttk.Combobox(header, values=[f"{i+1}" for i in range(16)], width=5, state="readonly")
        self.cb_channel.current(0)
        self.cb_channel.pack(side=tk.LEFT)
        
        tk.Label(header, text="Trigger Type").pack(side=tk.LEFT, padx=(15,5))
        self.cb_edge = ttk.Combobox(header, values=["Falling", "Rising"], width=8, state="readonly")
        # Mặc định Falling: trigger vào start-bit UART = đầu lịch burst ESP32.
        self.cb_edge.current(0)
        self.cb_edge.pack(side=tk.LEFT)

        tk.Label(header, text="Precapture size").pack(side=tk.LEFT, padx=(15,5))
        self.cb_precap = ttk.Combobox(header, values=["512", "1024", "2048", "4096", "8190"], width=6, state="readonly")
        # Mặc định 1024: post-trigger ~7168 record ~ 2.2ms >= 1.6ms lịch burst
        # ESP32 -> 1 capture ôm trọn các giao thức (4096 hay cắt mất SPI/UART-B).
        self.cb_precap.current(1)
        self.cb_precap.pack(side=tk.LEFT)

        tk.Label(header, text="Sample Rate (Divisor)").pack(side=tk.LEFT, padx=(15,5))
        self.cb_scaler = ttk.Combobox(header, values=["0", "1", "3", "9", "49", "99", "499"], width=6, state="readonly")
        self.cb_scaler.current(1)
        self.cb_scaler.pack(side=tk.LEFT)
        
        # TÍNH NĂNG AUTO RUN (Real-time pipeline)
        self.auto_run_var = tk.BooleanVar(value=True)
        tk.Checkbutton(header, text="Auto Run", variable=self.auto_run_var, font=("Arial", 9, "bold"), fg="red").pack(side=tk.LEFT, padx=15)

        # TỰ ĐỘNG PREDICT + DECODE sau mỗi lần capture
        self.auto_decode_var = tk.BooleanVar(value=True)
        tk.Checkbutton(header, text="Auto Decode", variable=self.auto_decode_var, font=("Arial", 9, "bold"), fg="#0055aa").pack(side=tk.LEFT, padx=5)
        tk.Button(header, text="Decode Setup", command=self.open_decode_setup).pack(side=tk.LEFT, padx=5)

    def setup_waveform_window(self):
        body = tk.Frame(self, bg='white')
        body.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.waveform_frame = tk.Canvas(body, bg="grey")
        self.waveform_frame.pack(fill='both', expand=True)

    def setup_footer(self):
        footer = tk.Frame(self)
        footer.pack(fill=tk.X, side=tk.BOTTOM, padx=5, pady=5)
        
        tk.Label(footer, textvariable=self.time_diff_var, font='Helvetica 18 bold').pack(side=tk.LEFT, padx=(0, 30))
        tk.Label(footer, text="STATE:").pack(side=tk.LEFT)
        tk.Label(footer, textvariable=self.status_var).pack(side=tk.LEFT, padx=5)
        tk.Label(footer, textvariable=self.time_var).pack(side=tk.RIGHT, padx=20)

    # =========================================================
    # PLOT: KHỞI TẠO VÀ CẬP NHẬT TRỰC TIẾP LÊN ĐỒ THỊ
    # =========================================================
    def init_plot_window(self, name):
        if self.canvas is not None:
            self.canvas.get_tk_widget().pack_forget()
            self.canvas = None
        
        num_ch = self.core.num_channels
        x_axis = self.core.x_axis
        channel_data = self.core.channel_data
        
        if not x_axis: return
        
        self.total_time_units = x_axis[-1]
        self.zoom_left = 0
        self.zoom_right = self.total_time_units
        self.is_first_coord = True
        
        fig, axs = plt.subplots(num_ch, sharex=True, sharey=True)
        self.fig = fig
        self.axs = axs
        fig.suptitle('Source: ' + name)
        
        canvas = FigureCanvasTkAgg(fig, master=self.waveform_frame)
        canvas.get_tk_widget().pack(side='top', fill='both', expand=True)
        self.canvas = canvas

        self.overlay_artists = []   # figure mới -> overlay cũ vô hiệu
        self.step_lines = []
        for x in range(num_ch):
            # Sử dụng drawstyle='steps-post' để dễ dàng cập nhật dữ liệu (Pipeline)
            line, = axs[x].plot(x_axis, channel_data[x], drawstyle='steps-post', linewidth=1.2, color='#0055ff')
            self.step_lines.append(line)
            
            # Căn lề Tên kênh nằm ngang, không bị đè
            axs[x].set_ylabel(f'CH-{x+1}', rotation=0, labelpad=25, va='center', fontweight='bold')
            axs[x].set_yticks([0, 1])
            axs[x].set_ylim(YLIM_BOTTOM, YLIM_TOP)
        
        # Vẽ các vạch Trigger đỏ
        trigger_channel = self.cb_channel.current()
        while self.drawn_triggers < len(self.core.trigger_times):
            trig_time = self.core.trigger_times[self.drawn_triggers]
            axs[trigger_channel].axvline(x=trig_time, color='red', linestyle='--', linewidth=3)
            self.drawn_triggers += 1
        
        self.spos = Slider(
            plt.axes([0.2, 0.03, 0.65, 0.03]),
            'Pos', 0, max(self.total_time_units, 1),
            valinit=(self.core.trigger_times[-1] if self.core.trigger_times else self.total_time_units/2))
        
        self.btn = Button(plt.axes([0.0, 0.00, 0.1, 0.03]), 'Reset View')
        
        self.spos.on_changed(self._on_slider_update)
        self.btn.on_clicked(self._on_reset_view)
        fig.canvas.mpl_connect('button_press_event', self._on_click)
        fig.canvas.mpl_connect('scroll_event', self._on_scroll_zoom)
        
        if self.core.trigger_times:
            view_window = min(2000, self.total_time_units)
            center = self.core.trigger_times[-1]
            self.zoom_left = max(center - view_window / 2, 0)
            self.zoom_right = min(center + view_window / 2, self.total_time_units)
            self._apply_zoom()
        
        canvas.draw()

    def update_plot_window(self):
        """Cập nhật dữ liệu mới nối đuôi vào đồ thị hiện tại (In-place update)."""
        if self.axs is None: return
        x_axis = self.core.x_axis
        channel_data = self.core.channel_data
        if not x_axis: return
        
        old_max = self.total_time_units
        self.total_time_units = x_axis[-1]
        
        # SMART SCROLL: Nếu thanh cuộn đang ở sát lề phải, tự động trôi theo sóng mới
        is_tracking_edge = (self.zoom_right >= old_max - 50) 
        
        # Bơm mảng dữ liệu mới vào các đường Line hiện tại
        for x in range(self.core.num_channels):
            self.step_lines[x].set_data(x_axis, channel_data[x])
            
        # Vẽ thêm các vạch Trigger đỏ mới vừa sập bẫy
        trigger_channel = self.cb_channel.current()
        while self.drawn_triggers < len(self.core.trigger_times):
            trig_time = self.core.trigger_times[self.drawn_triggers]
            self.axs[trigger_channel].axvline(x=trig_time, color='red', linestyle='--', linewidth=3)
            self.drawn_triggers += 1
            
        # Cập nhật giới hạn Slider
        if self.spos:
            self.spos.valmax = max(self.total_time_units, 1)
            self.spos.ax.set_xlim(0, self.spos.valmax)
            
        # Dịch chuyển khung hình nếu đang Tracking
        if is_tracking_edge:
            window_size = self.zoom_right - self.zoom_left
            self.zoom_right = self.total_time_units
            self.zoom_left = max(0, self.zoom_right - window_size)
            if self.spos:
                self.spos.eventson = False # Tạm khóa callback để không giật lag
                self.spos.set_val(self.zoom_right - window_size/2)
                self.spos.eventson = True
                
        self._apply_zoom()

    # =========================================================
    # INTERACTIONS: Slider, Zoom, Click
    # =========================================================
    def _apply_zoom(self):
        if self.axs is None: return
        for x in range(self.core.num_channels):
            self.axs[x].axis([self.zoom_left, self.zoom_right, YLIM_BOTTOM, YLIM_TOP])
        self.fig.canvas.draw_idle()

    def _on_slider_update(self, val):
        """Slider trượt khung hình qua lại (Pan) siêu mượt."""
        if self.axs is None: return
        cur_xlim = self.axs[0].get_xlim()
        window_size = cur_xlim[1] - cur_xlim[0]
        
        self.zoom_left = max(0, val - window_size / 2)
        self.zoom_right = min(self.total_time_units, val + window_size / 2)
        self._apply_zoom()

    def _on_reset_view(self, event):
        if self.spos: self.spos.reset()
        self.zoom_left = 0
        self.zoom_right = self.total_time_units
        self._apply_zoom()

    def _on_scroll_zoom(self, event):
        if event is None or event.xdata is None: return
        diff_left = abs(self.zoom_left - event.xdata)
        diff_right = abs(self.zoom_right - event.xdata)
        if event.button == 'up':  zoom = 0.8
        elif event.button == 'down': zoom = 1.2
        else: return
        
        self.zoom_left = max(abs(event.xdata - (diff_left * zoom)), 0)
        self.zoom_right = min(abs(event.xdata + (diff_right * zoom)), self.total_time_units)
        
        # Cập nhật vị trí cục Slider
        if self.spos:
            self.spos.eventson = False
            self.spos.set_val((self.zoom_left + self.zoom_right)/2)
            self.spos.eventson = True
        self._apply_zoom()

    def _find_nearest_transition_point(self, chan, x_event_point):
        x_axis = self.core.x_axis
        channel_data = self.core.channel_data
        num_x_points = len(x_axis)
        
        x_event_point = int(x_event_point)
        transition_point = 0
        t_click = x_axis[0]
        for i, tstamp in enumerate(x_axis):
            if x_event_point <= tstamp:
                transition_point = i
                t_click = tstamp
                break
        
        if transition_point > 0 and abs(x_event_point - x_axis[transition_point - 1]) < abs(t_click - x_event_point):
            transition_point -= 1
            t_click = x_axis[transition_point]
        
        click_value = channel_data[chan][transition_point]
        left = transition_point - 1
        right = transition_point + 1
        
        while left >= 0 and right < num_x_points:
            t_left = x_axis[left]
            t_right = x_axis[right]
            if abs(t_click - t_left) < abs(t_click - t_right):
                if channel_data[chan][left] != click_value:
                    transition_point = left + 1
                    break
                left -= 1
            else:
                if channel_data[chan][right] != click_value:
                    transition_point = right
                    break
                right += 1
        return x_axis[transition_point]

    def _on_click(self, event):
        if not event.inaxes or self.axs is None: return
        if not self.core.x_axis: return
        
        channel_num = 0
        for i, ax in enumerate(self.axs):
            if event.inaxes == ax:
                channel_num = i
                break
        
        tp = self._find_nearest_transition_point(channel_num, event.xdata)
        
        if self.is_first_coord:
            self.is_first_coord = False
            self.x1 = tp
            for chan in range(self.core.num_channels):
                for line in self.axs[chan].get_lines()[1:]:
                    if line.get_color() == 'green': line.remove()
        else:
            self.is_first_coord = True
            self.x2 = tp
            
            ticks_diff = abs(self.x1 - self.x2)
            ns_diff = ticks_diff * self.core.scaler_val * 10
            
            if ns_diff < 1000: time_str = f"{ns_diff:.2f} ns"
            elif ns_diff < 1000000: time_str = f"{(ns_diff/1000):.2f} us"
            else: time_str = f"{(ns_diff/1000000):.2f} ms"
            
            self.time_diff_var.set(time_str)
        
        self.axs[channel_num].axvline(x=tp, color='green', linestyle='--', linewidth=2)
        self.fig.canvas.draw_idle()

    # =========================================================
    # LUỒNG THU THẬP LOGIC PIPELINE
    # =========================================================
    def start_capture(self):
        if self.capture_thread and self.capture_thread.is_alive(): return
        if not self.core.ser or not self.core.ser.is_open:
            messagebox.showwarning("Cảnh báo", "Hãy kết nối cổng Serial!")
            return
            
        self.stop_requested = False
        
        # Chỉ reset dữ liệu khi người dùng chủ động bấm nút Start mới
        self.core.channel_data = [[] for _ in range(16)]
        self.core.x_axis = []
        self.core.trigger_times = []
        self.core.last_timestamp = 0
        self.is_first_plot = True
        self.drawn_triggers = 0
        self.channel_roles = None       # capture mới -> dự đoán lại từ đầu
        self.role_accum.reset()         # xóa niềm tin cũ (phiên đo mới)
        self.capture_block_starts = []
        self.overlay_artists = []
        
        ch = self.cb_channel.current()
        edge = self.cb_edge.current()
        scaler = int(self.cb_scaler.get())
        precap = int(self.cb_precap.get())
        
        self.core.send_config(ch, edge, scaler, precap)
        self.capture_thread = threading.Thread(target=self._capture_worker, daemon=True)
        self.capture_thread.start()

    def stop_capture(self):
        self.stop_requested = True
        self.auto_run_var.set(False) 
        if self.core.ser and self.core.ser.is_open:
            self.core.ser.write(bytes([CMD_SET_STOP, 0x01]))
        self.status_var.set("STOPPED")

    def _capture_worker(self):
        ser = self.core.ser
        ser.timeout = 0.5

        try:
            while not self.stop_requested:
                ser.reset_input_buffer()
                ser.write(bytes([CMD_SET_ENABLE, 0x01]))
                self.after(0, lambda: self.status_var.set("WAITING"))

                triggered = False
                buffer_full = False

                t_start = time.time()
                while not self.stop_requested:
                    if ser.in_waiting > 0:
                        b = ser.read(1)[0]
                        if b == HDR_TRIGGERED:
                            triggered = True
                            self.after(0, lambda: self.status_var.set("TRIGGERED"))
                            break
                    else:
                        time.sleep(0.01)
                        if time.time() - t_start > 1.5:
                            break

                if self.stop_requested: return

                if not triggered:
                    ser.write(bytes([CMD_SET_STOP, 0x01]))
                    time.sleep(0.05)
                    ser.write(bytes([CMD_SET_STOP, 0x00]))
                else:
                    t_start = time.time()
                    while not self.stop_requested:
                        if ser.in_waiting > 0:
                            b = ser.read(1)[0]
                            if b == HDR_DONE:
                                buffer_full = True
                                break
                        else:
                            time.sleep(0.01)
                            if time.time() - t_start > 1.5:
                                ser.write(bytes([CMD_SET_STOP, 0x01]))
                                time.sleep(0.05)
                                ser.write(bytes([CMD_SET_STOP, 0x00]))
                                buffer_full = True
                                break
                                
                if self.stop_requested: return

                ser.write(bytes([CMD_SET_START_READ, 0x01]))
                
                raw_bytes = bytearray()
                while not self.stop_requested:
                    chunk = ser.read(4096)
                    if not chunk: break 
                    raw_bytes.extend(chunk)
                        
                ser.write(bytes([CMD_SET_START_READ, 0x00]))
                ser.write(bytes([CMD_SET_ENABLE, 0x00]))
                
                if len(raw_bytes) > 0:
                    self.parse_and_append_data(raw_bytes)
                
                if triggered and not self.auto_run_var.get():
                    print("[HỆ THỐNG] Đã bắt được Trigger. Tạm dừng Pipeline.", flush=True)
                    self.after(0, lambda: self.status_var.set("STOPPED"))
                    break 

        except Exception as e:
            print(f"[LỖI CRASH LUỒNG] {e}", flush=True)

    def open_decode_setup(self):
        """Dialog chỉnh tay vai trò 16 kênh (lưới an toàn khi auto-predict nhầm)."""
        if not self.core.x_axis:
            messagebox.showinfo("Decode Setup", "Chưa có dữ liệu. Hãy Start Capture trước.")
            return
        # Đảm bảo có dự đoán làm mặc định
        if self.channel_roles is None:
            sa = self._analysis_accessor()
            self.channel_roles = self.role_accum.observe(
                predict_roles(sa, uart_baud=UART_BAUD))

        win = tk.Toplevel(self)
        win.title("Decode Setup - Gán vai trò kênh")
        win.geometry("560x620")

        tk.Label(win, text="Kênh", width=6, font=("Arial", 9, "bold")).grid(row=0, column=0, padx=4, pady=4)
        tk.Label(win, text="Vai trò (role)", width=16, font=("Arial", 9, "bold")).grid(row=0, column=1)
        tk.Label(win, text="Group", width=6, font=("Arial", 9, "bold")).grid(row=0, column=2)
        tk.Label(win, text="Dự đoán (conf)", width=18, font=("Arial", 9, "bold")).grid(row=0, column=3)

        role_vars = []
        group_vars = []
        for r in self.channel_roles:
            row = r.ch + 1
            tk.Label(win, text=f"CH-{r.ch+1}").grid(row=row, column=0, padx=4)

            rv = tk.StringVar(value=r.role)
            cb = ttk.Combobox(win, textvariable=rv, values=ROLE_CHOICES,
                              width=14, state="readonly")
            cb.grid(row=row, column=1, padx=2, pady=1)
            role_vars.append(rv)

            gv = tk.StringVar(value=str(r.group_id) if r.group_id is not None else "0")
            sp = tk.Spinbox(win, from_=0, to=9, width=4, textvariable=gv)
            sp.grid(row=row, column=2, padx=2)
            group_vars.append(gv)

            tk.Label(win, text=f"{r.role}  ({r.confidence*100:.0f}%)",
                     fg="#555555").grid(row=row, column=3, sticky="w")

        def apply_and_close():
            new_roles = []
            # Đếm counter theo group để đánh số bit b0,b1... theo tần số
            for ch in range(self.core.num_channels):
                role = role_vars[ch].get()
                try:
                    gid = int(group_vars[ch].get())
                except ValueError:
                    gid = 0
                group_id = None if gid == 0 else gid
                label = ROLE_DEFAULT_LABEL.get(role, "unknown")
                new_roles.append(ChannelRole(ch, role, label, group_id, 1.0))
            self.channel_roles = new_roles
            # Gán tay = KHÓA kênh: các capture Auto Run sau không được ghi đè.
            self.role_accum.set_manual(new_roles)
            win.destroy()
            self.run_decode(predict=False)

        btnf = tk.Frame(win)
        btnf.grid(row=18, column=0, columnspan=4, pady=10)
        tk.Button(btnf, text="Áp dụng + Decode", command=apply_and_close,
                  bg="#0055aa", fg="white", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=6)
        tk.Button(btnf, text="Huỷ", command=win.destroy).pack(side=tk.LEFT, padx=6)
        tk.Label(win, text="Mẹo: SPI cùng 1 Group (SCK/MOSI/CS), I2C cùng Group (SDA/SCL),\n"
                           "các bit Counter cùng Group. Group=0 nghĩa là không nhóm.",
                 fg="#777777", justify="left").grid(row=19, column=0, columnspan=4, padx=8)

    def parse_and_append_data(self, raw_bytes):
        """Hàm siêu việt nối đuôi dữ liệu vào Pipeline."""
        self.capture_block_starts.append(len(self.core.x_axis))
        total_time = self.core.last_timestamp
        trigger_marked = False # Reset cờ Trigger cho riêng block byte hiện tại
        
        idx = 0
        while idx + 3 < len(raw_bytes):
            header = raw_bytes[idx]
            if header in [HDR_PRE_BUFF, HDR_POST_BUFF]:
                ts = raw_bytes[idx+1]
                ch_low = raw_bytes[idx+2]
                ch_high = raw_bytes[idx+3]
                
                total_time += ts
                self.core.x_axis.append(total_time)
                
                if header == HDR_POST_BUFF and not trigger_marked:
                    self.core.trigger_times.append(total_time)
                    trigger_marked = True
                
                ch_data = (ch_high << 8) | ch_low
                for i in range(16):
                    self.core.channel_data[i].append((ch_data >> i) & 1)
                idx += 4
            else:
                idx += 1
                
        self.core.last_timestamp = total_time
        self.after(0, self._on_data_ready)

    def _on_data_ready(self):
        if self.is_first_plot:
            self.init_plot_window('Enxor Live Pipeline')
            self.is_first_plot = False
        else:
            self.update_plot_window()

        # Tự động predict + decode MỌI capture: dự đoán mới được gộp vào
        # RoleAccumulator -> cửa sổ trượt burst không phá kết quả đúng trước đó,
        # cửa sổ sau bổ sung giao thức còn thiếu (hội tụ dần khi Auto Run).
        if self.auto_decode_var.get() and len(self.core.x_axis) > 8:
            try:
                self.run_decode(predict=True)
            except Exception as e:
                print(f"[DECODE] Bỏ qua lỗi decode: {e}", flush=True)

        if self.core.trigger_times:
            self.time_var.set(f"Trigger Events: {len(self.core.trigger_times)} | Total Ticks: {self.core.last_timestamp}")
        else:
            self.time_var.set(f"Live Ticks: {self.core.last_timestamp}")

    # =========================================================
    # DECODE / PREDICT (P3 <-> P4)
    # =========================================================
    def run_predict_and_decode(self):
        """Menu: RESET niềm tin + dự đoán lại từ đầu rồi decode."""
        if not self.core.x_axis:
            messagebox.showinfo("Decode", "Chưa có dữ liệu. Hãy Start Capture trước.")
            return
        self.role_accum.reset()
        self.channel_roles = None
        self.run_decode(predict=True)

    def _analysis_accessor(self):
        """SampleAccessor trên BLOCK capture CUỐI: Auto Run nối các capture vào
        1 timeline nhưng giữa 2 block là thời gian chết thật -> phân tích xuyên
        seam sẽ ra rác. Block quá nhỏ thì dùng toàn bộ."""
        i0 = self.capture_block_starts[-1] if self.capture_block_starts else 0
        if len(self.core.x_axis) - i0 < 8:
            i0 = 0
        return build_accessor_from_core(self.core, self.core.num_channels,
                                        start_index=i0)

    def run_decode(self, predict=True):
        """SampleAccessor (block cuối) -> predict + merge -> decode_all -> nhãn + overlay."""
        if self.axs is None or not self.core.x_axis:
            return
        sa = self._analysis_accessor()

        if predict or self.channel_roles is None:
            predicted = predict_roles(sa, uart_baud=UART_BAUD)
            self.channel_roles = self.role_accum.observe(predicted)
            names = ", ".join(f"CH{r.ch+1}:{r.role}({r.confidence*100:.0f}%)"
                              for r in self.channel_roles
                              if r.role not in ("IDLE", "UNKNOWN"))
            print(f"[PREDICT] {names}", flush=True)

        annotations = decode_all(sa, self.channel_roles, uart_baud=UART_BAUD)
        self.apply_channel_labels(self.channel_roles)
        self.render_annotations(sa, annotations)
        if self.canvas:
            self.canvas.draw_idle()

    def apply_channel_labels(self, roles):
        """Đặt tên kênh: CH-n (label conf%); UART hiện thêm baud đo được."""
        for r in roles:
            if r.ch < len(self.axs):
                label = r.label
                if r.role == "UART" and r.evidence and r.evidence.get("bit_ticks"):
                    baud = 1e8 / self.core.scaler_val / r.evidence["bit_ticks"]
                    label = f"uart-{baud/1000:.1f}k"
                if r.confidence > 0:
                    label = f"{label} {r.confidence*100:.0f}%"
                self.axs[r.ch].set_ylabel(
                    f'CH-{r.ch+1}\n({label})', rotation=0, labelpad=32,
                    va='center', fontweight='bold', fontsize=7)

    def clear_overlay_and_labels(self):
        for art in self.overlay_artists:
            try:
                art.remove()
            except Exception:
                pass
        self.overlay_artists = []
        if self.axs is not None:
            for x in range(self.core.num_channels):
                self.axs[x].set_ylabel(f'CH-{x+1}', rotation=0, labelpad=25,
                                       va='center', fontweight='bold', fontsize=8)
            if self.canvas:
                self.canvas.draw_idle()

    def render_annotations(self, sa, annotations):
        """Vẽ overlay: byte + marker START/STOP/ACK ngay trên waveform."""
        # Xóa overlay cũ
        for art in self.overlay_artists:
            try:
                art.remove()
            except Exception:
                pass
        self.overlay_artists = []

        if not annotations:
            return
        # Giới hạn số annotation để tránh lag khi Auto Run
        if len(annotations) > self.max_overlay:
            annotations = annotations[-self.max_overlay:]

        for a in annotations:
            if a.ch >= len(self.axs):
                continue
            ax = self.axs[a.ch]
            if a.kind in MARKER_STYLE:
                color, ls = MARKER_STYLE[a.kind]
                ln = ax.axvline(x=a.t_start, color=color, linestyle=ls,
                                linewidth=1.0, alpha=0.7)
                self.overlay_artists.append(ln)
                txt = ax.text(a.t_start, OVERLAY_Y, a.text, color=color, fontsize=6,
                              ha='center', va='top', clip_on=True)
                self.overlay_artists.append(txt)
            else:
                # byte / counter / clock: text ở giữa khoảng [t_start, t_end]
                mid = (a.t_start + a.t_end) / 2.0
                color = '#003366' if a.kind == 'byte' else '#886600'
                txt = ax.text(mid, OVERLAY_Y, a.text, color=color, fontsize=6.5,
                              ha='center', va='top', clip_on=True,
                              bbox=dict(boxstyle='round,pad=0.1', fc='#ffffcc',
                                        ec='none', alpha=0.75))
                self.overlay_artists.append(txt)

if __name__ == "__main__":
    app = EnxorMainApp()
    app.mainloop()