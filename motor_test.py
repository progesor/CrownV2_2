import customtkinter as ctk
import serial
import serial.tools.list_ports
import threading
import time
import math

class MotorControlApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Ertip Motor Kontrol - Dinamik Limitli Arayüz")
        self.geometry("680x750")
        
        self.serial_port = None
        self.is_connected = False
        
        # --- Port Seçimi ---
        self.port_var = ctk.StringVar(value="Port Seç")
        self.ports_combobox = ctk.CTkComboBox(self, values=self.get_ports(), variable=self.port_var)
        self.ports_combobox.grid(row=0, column=0, padx=20, pady=10)
        
        self.btn_connect = ctk.CTkButton(self, text="Bağlan", command=self.toggle_connection)
        self.btn_connect.grid(row=0, column=1, padx=20, pady=10)
        
        # --- DÜZ MOD KONTROLÜ ---
        self.frame_cont = ctk.CTkFrame(self)
        self.frame_cont.grid(row=1, column=0, columnspan=2, padx=20, pady=10, sticky="ew")
        
        self.lbl_rpm = ctk.CTkLabel(self.frame_cont, text="Düz Mod Hedef: 0 RPM", font=("Arial", 14))
        self.lbl_rpm.pack(pady=5)
        self.slider_rpm = ctk.CTkSlider(self.frame_cont, from_=0, to=35000, command=self.slider_event)
        self.slider_rpm.set(0)
        self.slider_rpm.pack(fill="x", padx=20, pady=5)
        
        self.btn_send_rpm = ctk.CTkButton(self.frame_cont, text="DÜZ DÖNÜŞ (CONT)", fg_color="green", command=self.send_rpm)
        self.btn_send_rpm.pack(pady=10)

        # --- OSİLASYON MODU KONTROLÜ ---
        self.frame_osc = ctk.CTkFrame(self)
        self.frame_osc.grid(row=2, column=0, columnspan=2, padx=20, pady=10, sticky="ew")
        
        # SİSTEM İVMESİ (Ana Sınır Belirleyici)
        self.lbl_sys_accel = ctk.CTkLabel(self.frame_osc, text="Sistem Donanım İvmesi: 150000 RPM/s", font=("Arial", 12, "bold"))
        self.lbl_sys_accel.grid(row=0, column=0, columnspan=2, pady=5)
        self.slider_sys_accel = ctk.CTkSlider(self.frame_osc, from_=10000, to=500000, command=self.sys_accel_event)
        self.slider_sys_accel.set(150000)
        self.slider_sys_accel.grid(row=1, column=0, columnspan=2, sticky="ew", padx=50, pady=5)

        # AÇI SEÇİMİ
        self.lbl_osc_deg = ctk.CTkLabel(self.frame_osc, text="Osilasyon Açısı: 180°", font=("Arial", 12))
        self.lbl_osc_deg.grid(row=2, column=0, padx=10, pady=5)
        self.slider_osc_deg = ctk.CTkSlider(self.frame_osc, from_=10, to=720, command=self.slider_deg_event)
        self.slider_osc_deg.set(180)
        self.slider_osc_deg.grid(row=3, column=0, padx=10, pady=5)

        # DİNAMİK LİMİTLİ RPM SEÇİMİ
        self.lbl_osc_rpm = ctk.CTkLabel(self.frame_osc, text="Maks Güvenli Hız: 1500 RPM", font=("Arial", 12))
        self.lbl_osc_rpm.grid(row=2, column=1, padx=10, pady=5)
        self.slider_osc_rpm = ctk.CTkSlider(self.frame_osc, from_=100, to=35000, command=self.slider_oscrpm_event)
        self.slider_osc_rpm.set(1500)
        self.slider_osc_rpm.grid(row=3, column=1, padx=10, pady=5)

        self.btn_send_osc = ctk.CTkButton(self.frame_osc, text="OSİLASYONU BAŞLAT / GÜNCELLE", fg_color="#8B008B", command=self.send_osc)
        self.btn_send_osc.grid(row=4, column=0, columnspan=2, pady=15)

        # --- ORTAK BUTONLAR ---
        self.btn_stop = ctk.CTkButton(self, text="ACİL DURDUR", fg_color="red", height=40, font=("Arial", 16, "bold"), command=self.stop_motor)
        self.btn_stop.grid(row=3, column=0, columnspan=2, pady=10)
        
        # --- TELEMETRİ EKRANI ---
        self.lbl_telemetry = ctk.CTkLabel(self, text="Anlık Durum Bekleniyor...", font=("Courier", 14), justify="left")
        self.lbl_telemetry.grid(row=4, column=0, columnspan=2, pady=10)

        # Arayüz açıldığında ilk limitleri hesapla
        self.update_rpm_limits()

    def get_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def toggle_connection(self):
        if not self.is_connected:
            try:
                self.serial_port = serial.Serial(self.port_var.get(), 115200, timeout=0.1)
                self.is_connected = True
                self.btn_connect.configure(text="Kopar", fg_color="orange")
                threading.Thread(target=self.read_serial_loop, daemon=True).start()
            except Exception as e:
                print(f"Bağlantı hatası: {e}")
        else:
            self.stop_motor()
            self.is_connected = False
            if self.serial_port:
                self.serial_port.close()
            self.btn_connect.configure(text="Bağlan", fg_color=["#3B8ED0", "#1F6AA5"])

    def sys_accel_event(self, value):
        self.lbl_sys_accel.configure(text=f"Sistem Donanım İvmesi: {int(value)} RPM/s")
        self.update_rpm_limits()

    def slider_deg_event(self, value):
        self.lbl_osc_deg.configure(text=f"Osilasyon Açısı: {int(value)}°")
        self.update_rpm_limits()

    def slider_oscrpm_event(self, value):
        self.lbl_osc_rpm.configure(text=f"Maks Güvenli Hız: {int(value)} RPM")

    def update_rpm_limits(self):
        # Seçili ivme ve açıya göre fiziksel olarak ulaşılabilecek maks hızı hesapla
        max_accel_rpm_s = int(self.slider_sys_accel.get())
        deg = int(self.slider_osc_deg.get())
        
        # Formül: V_maks = sqrt(A_deg * Mesafe) (Hızlanıp yavaşlayacağı için mesafe D/2 alınmaz, formülden v = sqrt(a*d) çıkar)
        a_deg = max_accel_rpm_s * 6.0
        v_max_deg = math.sqrt(a_deg * deg)
        max_reachable_rpm = int(v_max_deg / 6.0)
        
        # RPM Sürgüsünün limitlerini güvenli bölgeye kilitle
        if max_reachable_rpm > 35000: max_reachable_rpm = 35000
        if max_reachable_rpm < 100: max_reachable_rpm = 100
        
        self.slider_osc_rpm.configure(to=max_reachable_rpm)
        
        # Eğer kullanıcının şu anki seçtiği RPM yeni limitten büyükse, sürgüyü aşağı çek
        if self.slider_osc_rpm.get() > max_reachable_rpm:
            self.slider_osc_rpm.set(max_reachable_rpm)
            self.lbl_osc_rpm.configure(text=f"Maks Güvenli Hız: {max_reachable_rpm} RPM")

    def slider_event(self, value):
        self.lbl_rpm.configure(text=f"Düz Mod Hedef: {int(value)} RPM")

    def send_rpm(self):
        rpm = int(self.slider_rpm.get())
        self.send_command("SET_RPM", rpm, 0, 0)

    def send_osc(self):
        deg = int(self.slider_osc_deg.get())
        rpm = int(self.slider_osc_rpm.get())
        sys_accel = int(self.slider_sys_accel.get())
        self.send_command("OSC", deg, rpm, sys_accel)

    def stop_motor(self):
        self.slider_rpm.set(0)
        self.lbl_rpm.configure(text="Düz Mod Hedef: 0 RPM")
        self.send_command("STOP", 0, 0, 0)

    def send_command(self, cmd, param1, param2, param3):
        if self.is_connected and self.serial_port:
            packet = f"<{cmd},{param1},{param2},{param3}>\n"
            self.serial_port.write(packet.encode('utf-8'))

    def read_serial_loop(self):
        buffer = ""
        while self.is_connected:
            try:
                if self.serial_port.in_waiting > 0:
                    buffer += self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    if '\n' in buffer:
                        lines = buffer.split('\n')
                        for line in lines[:-1]:
                            self.parse_telemetry(line.strip())
                        buffer = lines[-1]
            except:
                pass
            time.sleep(0.01)

    def parse_telemetry(self, line):
        if line.startswith("<TEL,") and line.endswith(">"):
            data = line[5:-1].split(',')
            if len(data) >= 4:
                act_rpm, act_pos, pwm, current = data[0], data[1], data[2], data[3]
                display_text = f"Anlık RPM: {act_rpm} | PWM: %{float(pwm)*100:.1f}\nAkım: {current}A | Konum: {act_pos}°"
                self.lbl_telemetry.configure(text=display_text)

if __name__ == "__main__":
    app = MotorControlApp()
    app.mainloop()