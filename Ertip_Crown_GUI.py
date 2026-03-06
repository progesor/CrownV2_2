import customtkinter as ctk
import serial
import serial.tools.list_ports
import struct
import threading
import time

# --- ARAYÜZ AYARLARI ---
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

class ErtipCrownApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Ertip Crown V1.3 - Geliştirici Arayüzü")
        self.geometry("850x550")
        
        # --- HABERLEŞME DEĞİŞKENLERİ ---
        self.ser = None
        self.is_connected = False
        self.target_rpm = 0.0
        self.motor_running = False
        
        # Thread Güvenliği ve Senkronizasyon
        self.ack_event = threading.Event()
        self.serial_lock = threading.Lock() # Çakışmaları önleyen kilit
        
        # --- İSTATİSTİK DEĞİŞKENLERİ ---
        self.stat_sent = 0
        self.stat_ack = 0
        self.stat_lost = 0

        self.create_widgets()
        
        # Heartbeat'i arayüzden bağımsız, asla donmayan ayrı bir Thread'e alıyoruz!
        threading.Thread(target=self.heartbeat_worker, daemon=True).start()

    def create_widgets(self):
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # === SOL PANEL (Bağlantı ve Canlı Telemetri) ===
        self.left_frame = ctk.CTkFrame(self, width=280, corner_radius=0)
        self.left_frame.grid(row=0, column=0, sticky="nsew")
        self.left_frame.grid_rowconfigure(8, weight=1)

        self.logo_label = ctk.CTkLabel(self.left_frame, text="ERTIP CROWN", font=ctk.CTkFont(size=20, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))

        # Port Seçimi
        self.port_combobox = ctk.CTkComboBox(self.left_frame, values=self.get_ports())
        self.port_combobox.grid(row=1, column=0, padx=20, pady=10)
        
        self.btn_connect = ctk.CTkButton(self.left_frame, text="Bağlan", command=self.toggle_connection)
        self.btn_connect.grid(row=2, column=0, padx=20, pady=10)

        # Telemetri Göstergeleri
        self.lbl_telemetry_title = ctk.CTkLabel(self.left_frame, text="Canlı Telemetri", font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_telemetry_title.grid(row=3, column=0, padx=20, pady=(20, 0))

        self.lbl_rpm = ctk.CTkLabel(self.left_frame, text="Hız: 0 RPM", font=ctk.CTkFont(size=16))
        self.lbl_rpm.grid(row=4, column=0, padx=20, pady=(5, 0), sticky="w")
        
        self.lbl_pwm = ctk.CTkLabel(self.left_frame, text="Güç (PWM): %0.0", font=ctk.CTkFont(size=16))
        self.lbl_pwm.grid(row=5, column=0, padx=20, pady=5, sticky="w")
        
        # YENİ: Paket İstatistik (Teşhis) Paneli
        self.stat_frame = ctk.CTkFrame(self.left_frame, fg_color="#2b2b2b")
        self.stat_frame.grid(row=6, column=0, padx=15, pady=15, sticky="ew")
        
        ctk.CTkLabel(self.stat_frame, text="Haberleşme Teşhisi", font=ctk.CTkFont(size=12, weight="bold")).pack(pady=(5,0))
        self.lbl_stat_sent = ctk.CTkLabel(self.stat_frame, text="Gönderilen: 0", font=ctk.CTkFont(size=11), text_color="#3498db")
        self.lbl_stat_sent.pack(anchor="w", padx=10)
        
        self.lbl_stat_ack = ctk.CTkLabel(self.stat_frame, text="Başarılı (ACK): 0", font=ctk.CTkFont(size=11), text_color="#2ecc71")
        self.lbl_stat_ack.pack(anchor="w", padx=10)
        
        self.lbl_stat_lost = ctk.CTkLabel(self.stat_frame, text="Kayıp/Timeout: 0", font=ctk.CTkFont(size=11), text_color="#e74c3c")
        self.lbl_stat_lost.pack(anchor="w", padx=10, pady=(0,5))

        self.lbl_status = ctk.CTkLabel(self.left_frame, text="Durum: Bekleniyor...", text_color="gray", font=ctk.CTkFont(size=12))
        self.lbl_status.grid(row=7, column=0, padx=20, pady=20, sticky="sw")

        # === SAĞ PANEL (Sekmeli Kontrol Sistemi) ===
        self.tabview = ctk.CTkTabview(self)
        self.tabview.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")
        
        self.tabview.add("Manuel Kontrol")
        self.tabview.add("PID Kalibrasyon")
        self.tabview.add("Reçete (Sequence)")

        self.setup_manual_tab()
        self.setup_recipe_tab()

    def setup_manual_tab(self):
        tab = self.tabview.tab("Manuel Kontrol")
        
        ctk.CTkLabel(tab, text="Hedef Hız (RPM):", font=ctk.CTkFont(size=16)).pack(pady=(20, 5))
        self.rpm_slider = ctk.CTkSlider(tab, from_=0, to=35000, number_of_steps=350, command=self.slider_event)
        self.rpm_slider.pack(pady=10, padx=20, fill="x")
        self.rpm_slider.set(1000)
        
        self.lbl_target_rpm = ctk.CTkLabel(tab, text="1000 RPM", font=ctk.CTkFont(size=18, weight="bold"))
        self.lbl_target_rpm.pack(pady=5)

        self.btn_set_rpm = ctk.CTkButton(tab, text="MOTORU ÇALIŞTIR", fg_color="green", hover_color="darkgreen", height=50, command=self.send_rpm)
        self.btn_set_rpm.pack(pady=20, padx=20, fill="x")

        self.btn_stop = ctk.CTkButton(tab, text="ACİL STOP", fg_color="red", hover_color="darkred", height=50, command=self.send_stop)
        self.btn_stop.pack(pady=10, padx=20, fill="x")

    def setup_recipe_tab(self):
        tab = self.tabview.tab("Reçete (Sequence)")
        ctk.CTkLabel(tab, text="Çok Adımlı Reçete Sistemi (Yakında Eklenecek)", text_color="gray").pack(pady=50)

    # --- DONANIM VE PROTOKOL FONKSİYONLARI ---
    def get_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports] if ports else ["Port Bulunamadı"]

    def slider_event(self, value):
        self.lbl_target_rpm.configure(text=f"{int(value)} RPM")

    def calculate_crc16(self, data: bytes) -> int:
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    def build_packet(self, cmd: int, payload: bytes = b"") -> bytes:
        length = len(payload)
        crc_data = struct.pack('<BB', length, cmd) + payload
        crc = self.calculate_crc16(crc_data)
        crc_l = crc & 0xFF
        crc_h = (crc >> 8) & 0xFF
        return struct.pack('<BB', 0xAA, 0x55) + crc_data + struct.pack('<BB', crc_l, crc_h)

    def toggle_connection(self):
        if not self.is_connected:
            port = self.port_combobox.get()
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.is_connected = True
                self.btn_connect.configure(text="Bağlantıyı Kes", fg_color="red")
                self.lbl_status.configure(text="Durum: Bağlandı", text_color="green")
                
                # İstatistikleri Sıfırla
                self.stat_sent = self.stat_ack = self.stat_lost = 0
                self.update_stats_ui()
                
                threading.Thread(target=self.serial_reader_thread, daemon=True).start()
            except Exception as e:
                self.lbl_status.configure(text=f"Hata: {e}", text_color="red")
        else:
            self.send_stop()
            self.is_connected = False
            if self.ser: self.ser.close()
            self.btn_connect.configure(text="Bağlan", fg_color=["#3a7ebf", "#1f538d"])
            self.lbl_status.configure(text="Durum: Bağlantı Kesildi", text_color="gray")

    def update_stats_ui(self):
        self.lbl_stat_sent.configure(text=f"Gönderilen: {self.stat_sent}")
        self.lbl_stat_ack.configure(text=f"Başarılı (ACK): {self.stat_ack}")
        self.lbl_stat_lost.configure(text=f"Kayıp/Timeout: {self.stat_lost}")

    def send_reliable(self, cmd: int, payload: bytes = b"", retries=3):
        def task():
            packet = self.build_packet(cmd, payload)
            for i in range(retries):
                if not self.is_connected: break
                
                self.ack_event.clear() 
                
                # UART gönderimini Thread-Safe yapıyoruz
                with self.serial_lock:
                    try:
                        self.ser.write(packet)
                        self.stat_sent += 1
                        self.after(0, self.update_stats_ui)
                    except:
                        break
                
                # 150 ms boyunca STM32'den "OK" cevabını bekle
                if self.ack_event.wait(0.15):
                    self.stat_ack += 1
                    self.after(0, self.update_stats_ui)
                    return # Onay geldi, görev başarılı!
                
                # Onay gelmedi, istatistiği artır ve tekrar dene
                self.stat_lost += 1
                self.after(0, self.update_stats_ui)
                time.sleep(0.05)
                
        threading.Thread(target=task, daemon=True).start()
        
    def send_rpm(self):
        if self.is_connected:
            self.target_rpm = float(self.rpm_slider.get())
            self.motor_running = True
            payload = struct.pack('<f', self.target_rpm)
            self.send_reliable(0x10, payload, retries=3)

    def send_stop(self):
        if self.is_connected:
            self.motor_running = False
            self.send_reliable(0x20, retries=3)

    # Arayüz donmasından bağımsız olarak çalışan Mükemmel Heartbeat
    def heartbeat_worker(self):
        while True:
            time.sleep(0.3) # 300ms Bekle
            if self.is_connected and self.motor_running:
                payload = struct.pack('<f', self.target_rpm)
                self.send_reliable(0x10, payload, retries=2) 

    def serial_reader_thread(self):
        while self.is_connected:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        if line.startswith("<TEL,"):
                            parts = line.strip("<>").split(",")
                            if len(parts) >= 5:
                                rpm = parts[1]
                                pwm = float(parts[3]) * 100.0 
                                self.lbl_rpm.configure(text=f"Hız: {rpm} RPM")
                                self.lbl_pwm.configure(text=f"Güç (PWM): %{pwm:.1f}")
                        
                        elif line.startswith("<DBG: CMD_RPM_OK") or line.startswith("<DBG: CMD_STOP_OK"):
                            self.ack_event.set() # Gönderici Thread'i uyandır
                            self.lbl_status.configure(text="Durum: Motor Çalışıyor", text_color="green")
                            
                        elif line.startswith("<DBG: WATCHDOG"):
                            self.lbl_status.configure(text="Durum: WATCHDOG DEVREDE!", text_color="red")
            except:
                pass

if __name__ == "__main__":
    app = ErtipCrownApp()
    app.mainloop()