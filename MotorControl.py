import customtkinter as ctk
import serial
import serial.tools.list_ports
import threading
import time
import math
import csv
from datetime import datetime

class MotorControlApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Ertip Motor Kontrol - Analiz ve Loglama Sürümü")
        self.geometry("700x800")
        
        self.serial_port = None
        self.is_connected = False
        
        # --- LOGLAMA DEĞİŞKENLERİ ---
        self.is_logging = False
        self.log_data = []
        self.start_time = 0.0
        # Anlık hedef değerleri hafızada tutuyoruz (Excel'de grafiği kolay çizmek için)
        self.cur_mode = "IDLE"
        self.cur_target_deg = 0
        self.cur_target_rpm = 0
        self.cur_target_accel = 0

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
        
        # SİSTEM İVMESİ
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

        # --- ORTAK BUTONLAR (DURDUR VE LOGLAMA) ---
        self.btn_stop = ctk.CTkButton(self, text="ACİL DURDUR", fg_color="red", height=40, font=("Arial", 14, "bold"), command=self.stop_motor)
        self.btn_stop.grid(row=3, column=0, padx=20, pady=10, sticky="ew")

        self.btn_log = ctk.CTkButton(self, text="LOGLAMAYI BAŞLAT", fg_color="#006400", height=40, font=("Arial", 14, "bold"), command=self.toggle_logging)
        self.btn_log.grid(row=3, column=1, padx=20, pady=10, sticky="ew")
        
        # --- TELEMETRİ EKRANI ---
        self.lbl_telemetry = ctk.CTkLabel(self, text="Anlık Durum Bekleniyor...", font=("Courier", 14), justify="left")
        self.lbl_telemetry.grid(row=4, column=0, columnspan=2, pady=10)

        # İlk hesaplamayı yap
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
            if self.is_logging: self.toggle_logging() # Bağlantı koparsa loglamayı bitir
            if self.serial_port:
                self.serial_port.close()
            self.btn_connect.configure(text="Bağlan", fg_color=["#3B8ED0", "#1F6AA5"])

    def toggle_logging(self):
        if not self.is_logging:
            self.is_logging = True
            self.btn_log.configure(text="DURDUR VE EXCEL'E KAYDET", fg_color="#FF8C00")
            self.log_data = []
            self.start_time = time.time()
            # CSV Başlıkları
            self.log_data.append([
                "Zaman(sn)", "Mod", 
                "Hedef_Aci", "Hedef_RPM", "Hedef_Ivme", 
                "Gercek_Aci", "Gercek_RPM", "PWM(%)", "Akim(A)"
            ])
            print("Loglama başladı...")
        else:
            self.is_logging = False
            self.btn_log.configure(text="LOGLAMAYI BAŞLAT", fg_color="#006400")
            self.save_log_to_csv()

    def save_log_to_csv(self):
        if len(self.log_data) <= 1:
            print("Kaydedilecek yeterli veri yok.")
            return
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"Motor_Log_{timestamp}.csv"
        
        try:
            with open(filename, mode='w', newline='', encoding='utf-8-sig') as file:
                writer = csv.writer(file, delimiter=';') # Excel'de sütunların düzgün ayrılması için noktalı virgül kullandık
                writer.writerows(self.log_data)
            print(f"BAŞARILI: Veriler {filename} dosyasına kaydedildi!")
            self.lbl_telemetry.configure(text=f"Son Kayıt: {filename}\nVeri Sayısı: {len(self.log_data)-1} satır")
        except Exception as e:
            print(f"Dosya kaydedilirken hata oluştu: {e}")

    def sys_accel_event(self, value):
        self.lbl_sys_accel.configure(text=f"Sistem Donanım İvmesi: {int(value)} RPM/s")
        self.update_rpm_limits()

    def slider_deg_event(self, value):
        self.lbl_osc_deg.configure(text=f"Osilasyon Açısı: {int(value)}°")
        self.update_rpm_limits()

    def slider_oscrpm_event(self, value):
        self.lbl_osc_rpm.configure(text=f"Maks Güvenli Hız: {int(value)} RPM")

    def update_rpm_limits(self):
        max_accel_rpm_s = int(self.slider_sys_accel.get())
        deg = int(self.slider_osc_deg.get())
        
        a_deg = max_accel_rpm_s * 6.0
        v_max_deg = math.sqrt(a_deg * deg)
        max_reachable_rpm = int(v_max_deg / 6.0)
        
        if max_reachable_rpm > 35000: max_reachable_rpm = 35000
        if max_reachable_rpm < 100: max_reachable_rpm = 100
        
        self.slider_osc_rpm.configure(to=max_reachable_rpm)
        
        if self.slider_osc_rpm.get() > max_reachable_rpm:
            self.slider_osc_rpm.set(max_reachable_rpm)
            self.lbl_osc_rpm.configure(text=f"Maks Güvenli Hız: {max_reachable_rpm} RPM")

    def slider_event(self, value):
        self.lbl_rpm.configure(text=f"Düz Mod Hedef: {int(value)} RPM")

    def send_rpm(self):
        rpm = int(self.slider_rpm.get())
        self.cur_mode = "CONT"
        self.cur_target_rpm = rpm
        self.cur_target_deg = 0
        self.cur_target_accel = 0
        self.send_command("SET_RPM", rpm, 0, 0)

    def send_osc(self):
        deg = int(self.slider_osc_deg.get())
        rpm = int(self.slider_osc_rpm.get())
        sys_accel = int(self.slider_sys_accel.get())
        
        self.cur_mode = "OSC"
        self.cur_target_deg = deg
        self.cur_target_rpm = rpm
        self.cur_target_accel = sys_accel
        self.send_command("OSC", deg, rpm, sys_accel)

    def stop_motor(self):
        self.slider_rpm.set(0)
        self.lbl_rpm.configure(text="Düz Mod Hedef: 0 RPM")
        
        self.cur_mode = "STOP"
        self.cur_target_deg = 0
        self.cur_target_rpm = 0
        self.cur_target_accel = 0
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
                act_rpm = float(data[0])
                act_pos = float(data[1])
                pwm = float(data[2])
                current = float(data[3])
                
                # Ekrana Yazdır
                display_text = f"Anlık RPM: {act_rpm} | PWM: %{pwm*100:.1f}\nAkım: {current}A | Konum: {act_pos}°"
                self.lbl_telemetry.configure(text=display_text)
                
                # Loglamaya Ekle
                if self.is_logging:
                    t_now = round(time.time() - self.start_time, 3)
                    self.log_data.append([
                        t_now, self.cur_mode,
                        self.cur_target_deg, self.cur_target_rpm, self.cur_target_accel,
                        round(act_pos, 2), round(act_rpm, 1), round(pwm*100, 1), round(current, 3)
                    ])

if __name__ == "__main__":
    app = MotorControlApp()
    app.mainloop()