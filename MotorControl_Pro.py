"""
Ertip Motor Kontrol - Gelişmiş Telemetri + Grafik Sürümü

Öne çıkanlar
- Thread-safe seri okuma (queue) + UI güncelleme (after)
- Canlı Matplotlib grafik (RPM/Pos/PWM/Current seçilebilir)
- "Sadece RPM" hızlı modu
- Kayan pencere (son X saniye) + otomatik ölçekleme
- Loglama (CSV, ; ayracı, UTF-8-SIG) + hedef değerler dahil
- Port yenileme, bağlantı durum göstergesi, hata güvenliği
- PID Canlı Ayarlama (Live Tuning)

Gereksinimler:
    pip install customtkinter pyserial matplotlib
"""

from __future__ import annotations

import customtkinter as ctk
import serial
import serial.tools.list_ports

import threading
import time
import math
import csv
from datetime import datetime
from dataclasses import dataclass
from collections import deque
from queue import Queue, Empty

# Matplotlib (TkAgg) embed
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


@dataclass
class TelemetrySample:
    t: float
    rpm: float
    pos_deg: float
    pwm_pct: float
    current_a: float


class MotorControlApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        ctk.set_appearance_mode("Dark")
        ctk.set_default_color_theme("blue")

        self.title("Ertip Motor Kontrol - Telemetri & Grafik")
        self.geometry("980x900")
        self.minsize(960, 850)

        # Serial
        self.serial_port: serial.Serial | None = None
        self.is_connected = False
        self.reader_thread: threading.Thread | None = None
        self.stop_reader = threading.Event()
        self.rx_queue: "Queue[str]" = Queue()

        # Loglama
        self.is_logging = False
        self.log_data: list[list] = []
        self.start_time = 0.0
        self.cur_mode = "IDLE"
        self.cur_target_deg = 0
        self.cur_target_rpm = 0
        self.cur_target_accel = 0

        # Grafik veri buffer (son N saniye)
        self.samples: deque[TelemetrySample] = deque(maxlen=6000)
        self.plot_paused = False
        self.window_seconds = ctk.DoubleVar(value=15.0)

        # ---------------- Layout root ----------------
        self.grid_columnconfigure(0, weight=0)  # Sol kontrol paneli
        self.grid_columnconfigure(1, weight=1)  # Sağ grafik paneli
        self.grid_rowconfigure(0, weight=1)

        self.left = ctk.CTkFrame(self, corner_radius=12)
        self.left.grid(row=0, column=0, sticky="nsw", padx=12, pady=12)
        self.left.grid_columnconfigure(0, weight=1)

        self.right = ctk.CTkFrame(self, corner_radius=12)
        self.right.grid(row=0, column=1, sticky="nsew", padx=(0, 12), pady=12)
        self.right.grid_rowconfigure(0, weight=1)
        self.right.grid_columnconfigure(0, weight=1)

        # ---------------- Sol panel: Port + kontrol ----------------
        self._build_port_section()
        self._build_control_sections()
        self._build_logging_section()
        self._build_telemetry_box()
        self._build_status_bar()

        # ---------------- Sağ panel: Grafik ----------------
        self._build_plot()

        # İlk hesap
        self.update_rpm_limits()

        # UI loop
        self.after(30, self._ui_loop)

        # Close handler
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- UI Builders ----------
    def _build_port_section(self):
        box = ctk.CTkFrame(self.left, corner_radius=10)
        box.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 8))
        box.grid_columnconfigure(0, weight=1)

        title = ctk.CTkLabel(box, text="Bağlantı", font=("Arial", 16, "bold"))
        title.grid(row=0, column=0, sticky="w", padx=10, pady=(8, 4))

        self.port_var = ctk.StringVar(value="Port Seç")
        self.ports_combobox = ctk.CTkComboBox(
            box, values=self.get_ports(), variable=self.port_var, state="readonly"
        )
        self.ports_combobox.grid(row=1, column=0, sticky="ew", padx=10, pady=6)

        btnrow = ctk.CTkFrame(box, fg_color="transparent")
        btnrow.grid(row=2, column=0, sticky="ew", padx=10, pady=(2, 10))
        btnrow.grid_columnconfigure((0, 1), weight=1)

        self.btn_refresh_ports = ctk.CTkButton(btnrow, text="Portları Yenile", command=self.refresh_ports)
        self.btn_refresh_ports.grid(row=0, column=0, sticky="ew", padx=(0, 6))

        self.btn_connect = ctk.CTkButton(btnrow, text="Bağlan", command=self.toggle_connection)
        self.btn_connect.grid(row=0, column=1, sticky="ew", padx=(6, 0))

    def _build_control_sections(self):
        # Düz mod (Row 1)
        cont = ctk.CTkFrame(self.left, corner_radius=10)
        cont.grid(row=1, column=0, sticky="ew", padx=10, pady=8)
        cont.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(cont, text="Düz Mod", font=("Arial", 15, "bold")).grid(row=0, column=0, sticky="w", padx=10, pady=(10, 6))
        self.lbl_rpm = ctk.CTkLabel(cont, text="Hedef: 0 RPM", font=("Arial", 13))
        self.lbl_rpm.grid(row=1, column=0, sticky="w", padx=10)

        self.slider_rpm = ctk.CTkSlider(cont, from_=0, to=35000, command=self.slider_event)
        self.slider_rpm.set(0)
        self.slider_rpm.grid(row=2, column=0, sticky="ew", padx=12, pady=(6, 10))

        self.btn_send_rpm = ctk.CTkButton(cont, text="DÜZ DÖNÜŞ (CONT)", fg_color="green", command=self.send_rpm)
        self.btn_send_rpm.grid(row=3, column=0, sticky="ew", padx=12, pady=(0, 12))

        # Osilasyon (Row 2)
        osc = ctk.CTkFrame(self.left, corner_radius=10)
        osc.grid(row=2, column=0, sticky="ew", padx=10, pady=8)
        osc.grid_columnconfigure((0, 1), weight=1)

        ctk.CTkLabel(osc, text="Osilasyon Modu", font=("Arial", 15, "bold")).grid(row=0, column=0, columnspan=2, sticky="w", padx=10, pady=(10, 6))

        self.lbl_sys_accel = ctk.CTkLabel(osc, text="Donanım İvmesi: 150000 RPM/s", font=("Arial", 12, "bold"))
        self.lbl_sys_accel.grid(row=1, column=0, columnspan=2, sticky="w", padx=10)

        self.slider_sys_accel = ctk.CTkSlider(osc, from_=10000, to=500000, command=self.sys_accel_event)
        self.slider_sys_accel.set(150000)
        self.slider_sys_accel.grid(row=2, column=0, columnspan=2, sticky="ew", padx=12, pady=(4, 10))

        self.lbl_osc_deg = ctk.CTkLabel(osc, text="Açı: 180°", font=("Arial", 12))
        self.lbl_osc_deg.grid(row=3, column=0, sticky="w", padx=12)

        self.lbl_osc_rpm = ctk.CTkLabel(osc, text="Maks Güvenli: 1500 RPM", font=("Arial", 12))
        self.lbl_osc_rpm.grid(row=3, column=1, sticky="w", padx=12)

        self.slider_osc_deg = ctk.CTkSlider(osc, from_=10, to=720, command=self.slider_deg_event)
        self.slider_osc_deg.set(180)
        self.slider_osc_deg.grid(row=4, column=0, sticky="ew", padx=12, pady=(4, 10))

        self.slider_osc_rpm = ctk.CTkSlider(osc, from_=100, to=35000, command=self.slider_oscrpm_event)
        self.slider_osc_rpm.set(1500)
        self.slider_osc_rpm.grid(row=4, column=1, sticky="ew", padx=12, pady=(4, 10))

        self.btn_send_osc = ctk.CTkButton(osc, text="OSİLASYONU BAŞLAT / GÜNCELLE", fg_color="#8B008B", command=self.send_osc)
        self.btn_send_osc.grid(row=5, column=0, columnspan=2, sticky="ew", padx=12, pady=(0, 12))

        # PID Ayarları (Row 3)
        pid = ctk.CTkFrame(self.left, corner_radius=10)
        pid.grid(row=3, column=0, sticky="ew", padx=10, pady=8)
        pid.grid_columnconfigure((0, 1), weight=1)

        ctk.CTkLabel(pid, text="Hız PID Ayarları", font=("Arial", 15, "bold")).grid(row=0, column=0, columnspan=2, sticky="w", padx=10, pady=(10, 6))

        self.lbl_kp = ctk.CTkLabel(pid, text="Kp (Oransal): 2.0", font=("Arial", 12))
        self.lbl_kp.grid(row=1, column=0, sticky="w", padx=12)

        self.lbl_ki = ctk.CTkLabel(pid, text="Ki (İntegral): 5.0", font=("Arial", 12))
        self.lbl_ki.grid(row=1, column=1, sticky="w", padx=12)

        # Kp limitleri: 0.0 - 20.0 (hassasiyet: 200 adım -> 0.1 artış)
        self.slider_kp = ctk.CTkSlider(pid, from_=0.0, to=20.0, number_of_steps=200, command=self.slider_kp_event)
        self.slider_kp.set(2.0)
        self.slider_kp.grid(row=2, column=0, sticky="ew", padx=12, pady=(4, 10))

        # Ki limitleri: 0.0 - 50.0 (hassasiyet: 500 adım -> 0.1 artış)
        self.slider_ki = ctk.CTkSlider(pid, from_=0.0, to=50.0, number_of_steps=500, command=self.slider_ki_event)
        self.slider_ki.set(5.0)
        self.slider_ki.grid(row=2, column=1, sticky="ew", padx=12, pady=(4, 10))

        self.btn_send_pid = ctk.CTkButton(pid, text="PID GÜNCELLE", fg_color="#1E90FF", command=self.send_pid)
        self.btn_send_pid.grid(row=3, column=0, columnspan=2, sticky="ew", padx=12, pady=(0, 12))

        # Acil durdur (Row 4)
        self.btn_stop = ctk.CTkButton(self.left, text="ACİL DURDUR", fg_color="red",
                                      height=42, font=("Arial", 15, "bold"),
                                      command=self.stop_motor)
        self.btn_stop.grid(row=4, column=0, sticky="ew", padx=10, pady=(8, 4))

    def _build_logging_section(self):
        box = ctk.CTkFrame(self.left, corner_radius=10)
        box.grid(row=5, column=0, sticky="ew", padx=10, pady=8)
        box.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(box, text="Kayıt (CSV)", font=("Arial", 15, "bold")).grid(row=0, column=0, sticky="w", padx=10, pady=(10, 6))

        self.btn_log = ctk.CTkButton(box, text="LOGLAMAYI BAŞLAT", fg_color="#006400", height=40,
                                     font=("Arial", 14, "bold"), command=self.toggle_logging)
        self.btn_log.grid(row=1, column=0, sticky="ew", padx=12, pady=(0, 8))

        self.lbl_last_log = ctk.CTkLabel(box, text="Son kayıt: -", font=("Arial", 11), text_color="#a8a8a8", wraplength=280, justify="left")
        self.lbl_last_log.grid(row=2, column=0, sticky="w", padx=12, pady=(0, 10))

    def _build_telemetry_box(self):
        box = ctk.CTkFrame(self.left, corner_radius=10)
        box.grid(row=6, column=0, sticky="ew", padx=10, pady=8)
        box.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(box, text="Anlık Telemetri", font=("Arial", 15, "bold")).grid(row=0, column=0, sticky="w", padx=10, pady=(10, 6))
        self.lbl_telemetry = ctk.CTkLabel(box, text="Bekleniyor...", font=("Courier", 13), justify="left")
        self.lbl_telemetry.grid(row=1, column=0, sticky="ew", padx=12, pady=(0, 12))

    def _build_status_bar(self):
        bar = ctk.CTkFrame(self.left, corner_radius=10)
        bar.grid(row=7, column=0, sticky="ew", padx=10, pady=(8, 10))
        bar.grid_columnconfigure(0, weight=1)

        self.lbl_status = ctk.CTkLabel(bar, text="Durum: Bağlı değil", font=("Arial", 12, "bold"))
        self.lbl_status.grid(row=0, column=0, sticky="w", padx=12, pady=10)

    def _build_plot(self):
        plotwrap = ctk.CTkFrame(self.right, corner_radius=10)
        plotwrap.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        plotwrap.grid_rowconfigure(1, weight=1)
        plotwrap.grid_columnconfigure(0, weight=1)

        # Üst kontrol bar
        ctrl = ctk.CTkFrame(plotwrap, fg_color="transparent")
        ctrl.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 4))
        ctrl.grid_columnconfigure(5, weight=1)

        ctk.CTkLabel(ctrl, text="Canlı Grafik", font=("Arial", 16, "bold")).grid(row=0, column=0, sticky="w")

        # Seçimler
        self.show_rpm = ctk.BooleanVar(value=True)
        self.show_pos = ctk.BooleanVar(value=False)
        self.show_pwm = ctk.BooleanVar(value=False)
        self.show_cur = ctk.BooleanVar(value=False)

        ctk.CTkCheckBox(ctrl, text="RPM", variable=self.show_rpm, command=self._on_plot_toggle).grid(row=0, column=1, padx=(12, 4))
        ctk.CTkCheckBox(ctrl, text="Pos(°)", variable=self.show_pos, command=self._on_plot_toggle).grid(row=0, column=2, padx=4)
        ctk.CTkCheckBox(ctrl, text="PWM(%)", variable=self.show_pwm, command=self._on_plot_toggle).grid(row=0, column=3, padx=4)
        ctk.CTkCheckBox(ctrl, text="Akım(A)", variable=self.show_cur, command=self._on_plot_toggle).grid(row=0, column=4, padx=4)

        # Sağ butonlar
        self.btn_only_rpm = ctk.CTkButton(ctrl, text="Sadece RPM", width=110, command=self._only_rpm)
        self.btn_only_rpm.grid(row=0, column=6, padx=(8, 4), sticky="e")

        self.btn_pause_plot = ctk.CTkButton(ctrl, text="Grafiği Durdur", width=120, command=self._toggle_plot_pause)
        self.btn_pause_plot.grid(row=0, column=7, padx=4, sticky="e")

        self.btn_clear = ctk.CTkButton(ctrl, text="Temizle", width=90, command=self._clear_samples)
        self.btn_clear.grid(row=0, column=8, padx=(4, 0), sticky="e")

        # Pencere kontrolü
        winrow = ctk.CTkFrame(plotwrap, fg_color="transparent")
        winrow.grid(row=2, column=0, sticky="ew", padx=10, pady=(4, 10))
        winrow.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(winrow, text="Zaman penceresi (sn):", font=("Arial", 12)).grid(row=0, column=0, sticky="w")
        self.slider_window = ctk.CTkSlider(winrow, from_=5, to=60, number_of_steps=55, variable=self.window_seconds)
        self.slider_window.grid(row=0, column=1, sticky="ew", padx=10)
        self.lbl_window = ctk.CTkLabel(winrow, text="15", width=30)
        self.lbl_window.grid(row=0, column=2, sticky="e")
        self.window_seconds.trace_add("write", lambda *_: self.lbl_window.configure(text=str(int(self.window_seconds.get()))))

        # Figure
        self.fig = Figure(figsize=(7.2, 5.6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("t (sn)")
        self.ax.set_ylabel("Değer")
        self.ax.grid(True, alpha=0.3)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plotwrap)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)

        self.line_rpm = None
        self.line_pos = None
        self.line_pwm = None
        self.line_cur = None

        self._rebuild_lines()

    # ---------- Ports ----------
    def get_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports] or ["Port bulunamadı"]

    def refresh_ports(self):
        ports = self.get_ports()
        self.ports_combobox.configure(values=ports)
        if ports and ports[0] != "Port bulunamadı":
            self.port_var.set(ports[0])

    # ---------- Connection ----------
    def toggle_connection(self):
        if not self.is_connected:
            port = self.port_var.get()
            if not port or port == "Port Seç" or port == "Port bulunamadı":
                self._set_status("Durum: Port seçilmedi", warn=True)
                return
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=0.1)
                self.is_connected = True
                self.stop_reader.clear()
                self.reader_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
                self.reader_thread.start()
                self.btn_connect.configure(text="Kopar", fg_color="orange")
                self._set_status(f"Durum: Bağlı ({port} @115200)")

                # Bağlanır bağlanmaz PID ayarlarını motora otomatik gönder
                self.after(200, self.send_pid)
            except Exception as e:
                self._set_status(f"Durum: Bağlantı hatası: {e}", warn=True)
        else:
            self.disconnect()

    def disconnect(self):
        try:
            self.stop_motor()
        except Exception:
            pass

        self.is_connected = False
        self.stop_reader.set()

        if self.is_logging:
            try:
                self.toggle_logging()
            except Exception:
                pass

        try:
            if self.serial_port:
                self.serial_port.close()
        except Exception:
            pass

        self.serial_port = None
        self.btn_connect.configure(text="Bağlan", fg_color=["#3B8ED0", "#1F6AA5"])
        self._set_status("Durum: Bağlı değil", warn=True)

    # ---------- Logging ----------
    def toggle_logging(self):
        if not self.is_logging:
            self.is_logging = True
            self.btn_log.configure(text="DURDUR VE KAYDET", fg_color="#FF8C00")
            self.log_data = []
            self.start_time = time.time()
            self.log_data.append([
                "Zaman(sn)", "Mod",
                "Hedef_Aci", "Hedef_RPM", "Hedef_Ivme",
                "Gercek_Aci", "Gercek_RPM", "PWM(%)", "Akim(A)"
            ])
            self.lbl_last_log.configure(text="Kayıt: başladı...")
        else:
            self.is_logging = False
            self.btn_log.configure(text="LOGLAMAYI BAŞLAT", fg_color="#006400")
            self.save_log_to_csv()

    def save_log_to_csv(self):
        if len(self.log_data) <= 1:
            self.lbl_last_log.configure(text="Kayıt: yeterli veri yok.")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"Motor_Log_{timestamp}.csv"
        try:
            with open(filename, mode="w", newline="", encoding="utf-8-sig") as f:
                writer = csv.writer(f, delimiter=";")
                writer.writerows(self.log_data)
            self.lbl_last_log.configure(text=f"Son kayıt: {filename}\nSatır: {len(self.log_data)-1}")
            self._set_status(f"Durum: Kayıt kaydedildi ({filename})")
        except Exception as e:
            self.lbl_last_log.configure(text=f"Kayıt: hata: {e}")
            self._set_status(f"Durum: Kayıt hatası: {e}", warn=True)

    # ---------- Osc limit calc ----------
    def sys_accel_event(self, value):
        self.lbl_sys_accel.configure(text=f"Donanım İvmesi: {int(value)} RPM/s")
        self.update_rpm_limits()

    def slider_deg_event(self, value):
        self.lbl_osc_deg.configure(text=f"Açı: {int(value)}°")
        self.update_rpm_limits()

    def slider_oscrpm_event(self, value):
        self.lbl_osc_rpm.configure(text=f"Maks Güvenli: {int(value)} RPM")

    def slider_kp_event(self, value):
        self.lbl_kp.configure(text=f"Kp (Oransal): {value:.1f}")

    def slider_ki_event(self, value):
        self.lbl_ki.configure(text=f"Ki (İntegral): {value:.1f}")

    def update_rpm_limits(self):
        max_accel_rpm_s = int(self.slider_sys_accel.get())
        deg = int(self.slider_osc_deg.get())

        a_deg = max_accel_rpm_s * 6.0
        v_max_deg = math.sqrt(max(a_deg * deg, 0.0))
        max_reachable_rpm = int(v_max_deg / 6.0)

        max_reachable_rpm = max(100, min(35000, max_reachable_rpm))
        self.slider_osc_rpm.configure(to=max_reachable_rpm)

        if self.slider_osc_rpm.get() > max_reachable_rpm:
            self.slider_osc_rpm.set(max_reachable_rpm)
            self.lbl_osc_rpm.configure(text=f"Maks Güvenli: {max_reachable_rpm} RPM")

    # ---------- Commands ----------
    def slider_event(self, value):
        self.lbl_rpm.configure(text=f"Hedef: {int(value)} RPM")

    def send_rpm(self):
        rpm = int(self.slider_rpm.get())
        self.cur_mode = "CONT"
        self.cur_target_rpm = rpm
        self.cur_target_deg = 0
        self.cur_target_accel = 0
        self.send_command("SET_RPM", float(rpm), 0.0, 0.0)

    def send_osc(self):
        deg = int(self.slider_osc_deg.get())
        rpm = int(self.slider_osc_rpm.get())
        sys_accel = int(self.slider_sys_accel.get())

        self.cur_mode = "OSC"
        self.cur_target_deg = deg
        self.cur_target_rpm = rpm
        self.cur_target_accel = sys_accel
        self.send_command("OSC", float(deg), float(rpm), float(sys_accel))

    def send_pid(self):
        kp = float(self.slider_kp.get())
        ki = float(self.slider_ki.get())
        self.send_command("PID", kp, ki, 0.0)

    def stop_motor(self):
        self.slider_rpm.set(0)
        self.lbl_rpm.configure(text="Hedef: 0 RPM")

        self.cur_mode = "STOP"
        self.cur_target_deg = 0
        self.cur_target_rpm = 0
        self.cur_target_accel = 0
        self.send_command("STOP", 0.0, 0.0, 0.0)

    def send_command(self, cmd: str, p1: float, p2: float, p3: float):
        if self.is_connected and self.serial_port:
            packet = f"<{cmd},{p1},{p2},{p3}>\n"
            try:
                self.serial_port.write(packet.encode("utf-8"))
            except Exception as e:
                self._set_status(f"Durum: gönderim hatası: {e}", warn=True)

    # ---------- Serial read thread ----------
    def read_serial_loop(self):
        buffer = ""
        while self.is_connected and not self.stop_reader.is_set():
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    chunk = self.serial_port.read(self.serial_port.in_waiting)
                    try:
                        buffer += chunk.decode("utf-8", errors="ignore")
                    except Exception:
                        buffer += chunk.decode(errors="ignore")

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if line:
                            self.rx_queue.put(line)
            except Exception:
                pass
            time.sleep(0.005)

    # ---------- UI loop (main thread) ----------
    def _ui_loop(self):
        drained = 0
        while drained < 200:
            try:
                line = self.rx_queue.get_nowait()
            except Empty:
                break
            drained += 1
            self.parse_telemetry(line)

        if not self.plot_paused:
            self._update_plot()

        self.after(40, self._ui_loop)

    # ---------- Telemetry parsing ----------
    def parse_telemetry(self, line: str):
        if not (line.startswith("<TEL,") and line.endswith(">")):
            return
        try:
            payload = line[5:-1]
            parts = payload.split(",")
            if len(parts) < 4:
                return

            act_rpm = float(parts[0])
            act_pos = float(parts[1])
            pwm = float(parts[2])
            current = float(parts[3])

            pwm_pct = pwm * 100.0 if pwm <= 1.5 else pwm

            t_now = time.time()
            if self.start_time <= 0:
                self.start_time = t_now
            t_rel = t_now - self.start_time

            sample = TelemetrySample(
                t=t_rel,
                rpm=act_rpm,
                pos_deg=act_pos,
                pwm_pct=pwm_pct,
                current_a=current,
            )
            self.samples.append(sample)

            self.lbl_telemetry.configure(
                text=f"RPM: {act_rpm:8.1f}\nPWM: {pwm_pct:7.1f} %\nAkım: {current:7.3f} A\nKonum: {act_pos:7.2f} °"
            )

            if self.is_logging:
                self.log_data.append([
                    round(t_rel, 3), self.cur_mode,
                    self.cur_target_deg, self.cur_target_rpm, self.cur_target_accel,
                    round(act_pos, 2), round(act_rpm, 1), round(pwm_pct, 1), round(current, 3)
                ])
        except Exception:
            return

    # ---------- Plot controls ----------
    def _on_plot_toggle(self):
        self._rebuild_lines()

    def _only_rpm(self):
        self.show_rpm.set(True)
        self.show_pos.set(False)
        self.show_pwm.set(False)
        self.show_cur.set(False)
        self._rebuild_lines()

    def _toggle_plot_pause(self):
        self.plot_paused = not self.plot_paused
        self.btn_pause_plot.configure(text=("Grafiği Devam Ettir" if self.plot_paused else "Grafiği Durdur"))

    def _clear_samples(self):
        self.samples.clear()
        self._update_plot(force_clear=True)

    def _rebuild_lines(self):
        self.ax.cla()
        self.ax.set_xlabel("t (sn)")
        self.ax.set_ylabel("Değer")
        self.ax.grid(True, alpha=0.3)

        self.line_rpm = self.ax.plot([], [], label="RPM")[0] if self.show_rpm.get() else None
        self.line_pos = self.ax.plot([], [], label="Pos(°)")[0] if self.show_pos.get() else None
        self.line_pwm = self.ax.plot([], [], label="PWM(%)")[0] if self.show_pwm.get() else None
        self.line_cur = self.ax.plot([], [], label="Akım(A)")[0] if self.show_cur.get() else None

        if any(v.get() for v in (self.show_rpm, self.show_pos, self.show_pwm, self.show_cur)):
            self.ax.legend(loc="upper right")

        self.canvas.draw_idle()

    def _update_plot(self, force_clear: bool = False):
        if force_clear or not self.samples:
            self.ax.set_xlim(0, max(1, int(self.window_seconds.get())))
            self.ax.set_ylim(0, 1)
            self.canvas.draw_idle()
            return

        win = float(self.window_seconds.get())
        t_max = self.samples[-1].t
        t_min = max(0.0, t_max - win)

        xs = []
        yrpm = []
        ypos = []
        ypwm = []
        ycur = []

        for s in self.samples:
            if s.t < t_min:
                continue
            xs.append(s.t)
            yrpm.append(s.rpm)
            ypos.append(s.pos_deg)
            ypwm.append(s.pwm_pct)
            ycur.append(s.current_a)

        if not xs:
            return

        y_all = []

        def set_line(line, y):
            if line is None:
                return
            line.set_data(xs, y)

        if self.line_rpm is not None:
            set_line(self.line_rpm, yrpm)
            y_all += yrpm
        if self.line_pos is not None:
            set_line(self.line_pos, ypos)
            y_all += ypos
        if self.line_pwm is not None:
            set_line(self.line_pwm, ypwm)
            y_all += ypwm
        if self.line_cur is not None:
            set_line(self.line_cur, ycur)
            y_all += ycur

        self.ax.set_xlim(t_min, t_max if t_max > t_min else t_min + 1.0)

        y_min = min(y_all)
        y_max = max(y_all)
        if abs(y_max - y_min) < 1e-6:
            y_max = y_min + 1.0
        pad = 0.08 * (y_max - y_min)
        self.ax.set_ylim(y_min - pad, y_max + pad)

        self.canvas.draw_idle()

    # ---------- Helpers ----------
    def _set_status(self, text: str, warn: bool = False):
        self.lbl_status.configure(text=text, text_color=("#ffb3b3" if warn else None))

    def on_close(self):
        try:
            self.disconnect()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    app = MotorControlApp()
    app.mainloop()