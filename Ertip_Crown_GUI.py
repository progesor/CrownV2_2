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
        self.geometry("850x600")
        
        # --- HABERLEŞME DEĞİŞKENLERİ ---
        self.ser = None
        self.is_connected = False
        self.target_rpm = 0.0
        self.motor_running = False
        self.current_mode = "IDLE"
        
        # Arayüz geçişleri için arka plan hafızası
        self.saved_osc_deg = 180.0
        self.saved_osc_time = 400.0
        
        # Yanıt (ACK) bekleme mekanizması
        self.ack_event = threading.Event()

        self.create_widgets()
        
        # Heartbeat ayrı bir thread'de donmadan çalışır
        threading.Thread(target=self.heartbeat_worker, daemon=True).start()

    def create_widgets(self):
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # === SOL PANEL (Bağlantı, Telemetri ve Hafıza) ===
        self.left_frame = ctk.CTkFrame(self, width=280, corner_radius=0)
        self.left_frame.grid(row=0, column=0, sticky="nsew")
        self.left_frame.grid_rowconfigure(7, weight=1)

        self.logo_label = ctk.CTkLabel(self.left_frame, text="ERTIP CROWN", font=ctk.CTkFont(size=20, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))

        # Port Seçimi
        self.port_combobox = ctk.CTkComboBox(self.left_frame, values=self.get_ports())
        self.port_combobox.grid(row=1, column=0, padx=20, pady=10)
        
        self.btn_connect = ctk.CTkButton(self.left_frame, text="Bağlan", command=self.toggle_connection)
        self.btn_connect.grid(row=2, column=0, padx=20, pady=10)

        # Telemetri Göstergeleri
        self.lbl_telemetry_title = ctk.CTkLabel(self.left_frame, text="Canlı Telemetri", font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_telemetry_title.grid(row=3, column=0, padx=20, pady=(15, 0))

        self.lbl_rpm = ctk.CTkLabel(self.left_frame, text="Hız: 0 RPM", font=ctk.CTkFont(size=16))
        self.lbl_rpm.grid(row=4, column=0, padx=20, pady=(5, 0), sticky="w")
        
        self.lbl_pwm = ctk.CTkLabel(self.left_frame, text="Güç (PWM): %0.0", font=ctk.CTkFont(size=16))
        self.lbl_pwm.grid(row=5, column=0, padx=20, pady=5, sticky="w")
        
        # --- YENİ: HAFIZA YÖNETİMİ PANELİ ---
        self.memory_frame = ctk.CTkFrame(self.left_frame, fg_color="#2b2b2b")
        self.memory_frame.grid(row=6, column=0, padx=15, pady=20, sticky="ew")
        
        ctk.CTkLabel(self.memory_frame, text="Cihaz Hafızası (EEPROM)", font=ctk.CTkFont(size=12, weight="bold")).pack(pady=(5,5))
        
        self.btn_get_params = ctk.CTkButton(self.memory_frame, text="🔄 Cihazdan Oku", height=30, fg_color="#2980b9", hover_color="#1f618d", command=self.send_get_params)
        self.btn_get_params.pack(pady=5, padx=10, fill="x")
        
        self.btn_save_params = ctk.CTkButton(self.memory_frame, text="💾 Kalıcı Kaydet", height=30, fg_color="#c0392b", hover_color="#922b21", command=self.send_save_params)
        self.btn_save_params.pack(pady=(0,10), padx=10, fill="x")

        self.lbl_status = ctk.CTkLabel(self.left_frame, text="Durum: Bekleniyor...", text_color="gray", font=ctk.CTkFont(size=12))
        self.lbl_status.grid(row=7, column=0, padx=20, pady=10, sticky="sw")

        # === SAĞ PANEL (Sekmeler) ===
        self.tabview = ctk.CTkTabview(self)
        self.tabview.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")
        
        self.tabview.add("Manuel Kontrol")
        self.tabview.add("PID Kalibrasyon")
        self.tabview.add("Reçete (Osilasyon)")

        self.setup_manual_tab()
        self.setup_pid_tab()
        self.setup_recipe_tab()

    # --- SEKME KURULUMLARI ---
    def setup_manual_tab(self):
        tab = self.tabview.tab("Manuel Kontrol")
        ctk.CTkLabel(tab, text="Hedef Hız (RPM):", font=ctk.CTkFont(size=16)).pack(pady=(20, 5))
        self.rpm_slider = ctk.CTkSlider(tab, from_=0, to=35000, number_of_steps=350, command=lambda v: self.lbl_target_rpm.configure(text=f"{int(v)} RPM"))
        self.rpm_slider.pack(pady=10, padx=20, fill="x")
        self.rpm_slider.set(1000)
        self.lbl_target_rpm = ctk.CTkLabel(tab, text="1000 RPM", font=ctk.CTkFont(size=18, weight="bold"))
        self.lbl_target_rpm.pack(pady=5)
        self.btn_set_rpm = ctk.CTkButton(tab, text="MOTORU ÇALIŞTIR", fg_color="green", hover_color="darkgreen", height=50, command=self.send_rpm)
        self.btn_set_rpm.pack(pady=20, padx=20, fill="x")
        self.btn_stop = ctk.CTkButton(tab, text="ACİL STOP", fg_color="red", hover_color="darkred", height=50, command=self.send_stop)
        self.btn_stop.pack(pady=10, padx=20, fill="x")

    def setup_pid_tab(self):
        tab = self.tabview.tab("PID Kalibrasyon")
        ctk.CTkLabel(tab, text="Oransal Kazanç (Kp):", font=ctk.CTkFont(size=14)).pack(pady=(10, 0))
        self.lbl_kp_val = ctk.CTkLabel(tab, text="1.00", font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_kp_val.pack()
        self.slider_kp = ctk.CTkSlider(tab, from_=0.0, to=20.0, command=lambda v: self.lbl_kp_val.configure(text=f"{v:.2f}"))
        self.slider_kp.set(1.0)
        self.slider_kp.pack(pady=5, padx=20, fill="x")

        ctk.CTkLabel(tab, text="İntegral Kazancı (Ki):", font=ctk.CTkFont(size=14)).pack(pady=(10, 0))
        self.lbl_ki_val = ctk.CTkLabel(tab, text="5.00", font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_ki_val.pack()
        self.slider_ki = ctk.CTkSlider(tab, from_=0.0, to=100.0, command=lambda v: self.lbl_ki_val.configure(text=f"{v:.2f}"))
        self.slider_ki.set(5.0)
        self.slider_ki.pack(pady=5, padx=20, fill="x")

        self.btn_send_pid = ctk.CTkButton(tab, text="CANLI PID TESTİ GÖNDER", fg_color="#e67e22", hover_color="#d35400", height=40, command=self.send_pid)
        self.btn_send_pid.pack(pady=20, padx=20, fill="x")

    def setup_recipe_tab(self):
        tab = self.tabview.tab("Reçete (Osilasyon)")
        
        # Mod Seçici
        self.osc_mode_var = ctk.StringVar(value="Zaman Tabanlı (Medikal Punch)")
        self.seg_btn = ctk.CTkSegmentedButton(tab, values=["Açı Tabanlı (Klasik)", "Zaman Tabanlı (Medikal Punch)"],
                                              variable=self.osc_mode_var, command=self.change_osc_mode_ui)
        self.seg_btn.pack(pady=(10, 15), padx=20, fill="x")

        # Birincil Değişken (Açı veya Süre)
        self.dynamic_lbl_title = ctk.CTkLabel(tab, text="Hareket Süresi - Wide (Milisaniye):", font=ctk.CTkFont(size=14))
        self.dynamic_lbl_title.pack(pady=(5, 0))
        self.lbl_osc_primary = ctk.CTkLabel(tab, text="400 ms", font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_osc_primary.pack()
        self.slider_osc_primary = ctk.CTkSlider(tab, from_=50.0, to=1000.0, command=self.update_primary_label)
        self.slider_osc_primary.set(400)
        self.slider_osc_primary.pack(pady=5, padx=20, fill="x")

        # Hız
        ctk.CTkLabel(tab, text="Hedef Hız - Strength (RPM):", font=ctk.CTkFont(size=14)).pack(pady=(5, 0))
        self.lbl_osc_rpm = ctk.CTkLabel(tab, text="2500 RPM", font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_osc_rpm.pack()
        self.slider_osc_rpm = ctk.CTkSlider(tab, from_=500.0, to=15000.0, command=lambda v: self.lbl_osc_rpm.configure(text=f"{int(v)} RPM"))
        self.slider_osc_rpm.set(2500)
        self.slider_osc_rpm.pack(pady=5, padx=20, fill="x")

        # İvme
        ctk.CTkLabel(tab, text="İvme - Punch Etkisi (RPM/s):", font=ctk.CTkFont(size=14)).pack(pady=(5, 0))
        self.lbl_osc_accel = ctk.CTkLabel(tab, text="15000", font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_osc_accel.pack()
        self.slider_osc_accel = ctk.CTkSlider(tab, from_=1000.0, to=30000.0, command=lambda v: self.lbl_osc_accel.configure(text=f"{int(v)}"))
        self.slider_osc_accel.set(15000)
        self.slider_osc_accel.pack(pady=5, padx=20, fill="x")

        self.btn_send_osc = ctk.CTkButton(tab, text="OSİLASYONU BAŞLAT", fg_color="#8e44ad", hover_color="#732d91", height=40, command=self.send_oscillation)
        self.btn_send_osc.pack(pady=15, padx=20, fill="x")

    # --- ARAYÜZ YARDIMCI FONKSİYONLARI ---
    def change_osc_mode_ui(self, value):
        if value == "Açı Tabanlı (Klasik)":
            self.dynamic_lbl_title.configure(text="Osilasyon Açısı (Derece):")
            self.slider_osc_primary.configure(from_=10.0, to=720.0)
            self.slider_osc_primary.set(self.saved_osc_deg)
            self.lbl_osc_primary.configure(text=f"{int(self.saved_osc_deg)} Derece")
        else:
            self.dynamic_lbl_title.configure(text="Hareket Süresi - Wide (Milisaniye):")
            self.slider_osc_primary.configure(from_=50.0, to=1000.0)
            self.slider_osc_primary.set(self.saved_osc_time)
            self.lbl_osc_primary.configure(text=f"{int(self.saved_osc_time)} ms")

    def update_primary_label(self, val):
        if self.osc_mode_var.get() == "Açı Tabanlı (Klasik)":
            self.saved_osc_deg = float(val)
            self.lbl_osc_primary.configure(text=f"{int(val)} Derece")
        else:
            self.saved_osc_time = float(val)
            self.lbl_osc_primary.configure(text=f"{int(val)} ms")

    # Cihazdan gelen parametreleri arayüze döşer
    def update_ui_from_params(self, kp, ki, osc_time, osc_deg, osc_rpm, osc_accel):
        self.slider_kp.set(kp)
        self.lbl_kp_val.configure(text=f"{kp:.2f}")
        self.slider_ki.set(ki)
        self.lbl_ki_val.configure(text=f"{ki:.2f}")

        self.slider_osc_rpm.set(osc_rpm)
        self.lbl_osc_rpm.configure(text=f"{int(osc_rpm)} RPM")
        self.slider_osc_accel.set(osc_accel)
        self.lbl_osc_accel.configure(text=f"{int(osc_accel)}")

        self.saved_osc_time = osc_time
        self.saved_osc_deg = osc_deg

        if self.osc_mode_var.get() == "Açı Tabanlı (Klasik)":
            self.slider_osc_primary.set(osc_deg)
            self.lbl_osc_primary.configure(text=f"{int(osc_deg)} Derece")
        else:
            self.slider_osc_primary.set(osc_time)
            self.lbl_osc_primary.configure(text=f"{int(osc_time)} ms")
            
        self.lbl_status.configure(text="Durum: Ayarlar Senkronize Edildi", text_color="#3498db")

    # --- DONANIM VE HABERLEŞME FONKSİYONLARI ---
    def get_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports] if ports else ["Port Bulunamadı"]

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
        return struct.pack('<BB', 0xAA, 0x55) + crc_data + struct.pack('<BB', crc & 0xFF, (crc >> 8) & 0xFF)

    def toggle_connection(self):
        if not self.is_connected:
            port = self.port_combobox.get()
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.is_connected = True
                self.btn_connect.configure(text="Bağlantıyı Kes", fg_color="red")
                self.lbl_status.configure(text="Durum: Bağlandı", text_color="green")
                threading.Thread(target=self.serial_reader_thread, daemon=True).start()
                
                # Cihaz bağlandığı an otomatik olarak hafızayı oku
                self.after(500, self.send_get_params)
            except Exception as e:
                self.lbl_status.configure(text=f"Hata: {e}", text_color="red")
        else:
            self.send_stop()
            self.is_connected = False
            if self.ser: self.ser.close()
            self.btn_connect.configure(text="Bağlan", fg_color=["#3a7ebf", "#1f538d"])
            self.lbl_status.configure(text="Durum: Bağlantı Kesildi", text_color="gray")

    def send_reliable(self, cmd: int, payload: bytes = b"", retries=3):
        def task():
            packet = self.build_packet(cmd, payload)
            for i in range(retries):
                if not self.is_connected: break
                self.ack_event.clear() 
                self.ser.write(packet)
                if self.ack_event.wait(0.15): return 
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

    def send_pid(self):
        if self.is_connected:
            kp = float(self.slider_kp.get())
            ki = float(self.slider_ki.get())
            payload = struct.pack('<ff', kp, ki)
            self.send_reliable(0x30, payload, retries=3)

    def send_oscillation(self):
        if self.is_connected:
            self.motor_running = True
            primary_val = float(self.slider_osc_primary.get()) 
            rpm = float(self.slider_osc_rpm.get())
            accel = float(self.slider_osc_accel.get())
            
            payload = struct.pack('<fff', primary_val, rpm, accel)
            cmd = 0x50 if self.osc_mode_var.get() == "Zaman Tabanlı (Medikal Punch)" else 0x40
            self.send_reliable(cmd, payload, retries=3)

    def send_get_params(self):
        if self.is_connected:
            self.lbl_status.configure(text="Durum: Ayarlar Çekiliyor...", text_color="orange")
            self.send_reliable(0x60, retries=3)

    def send_save_params(self):
        if self.is_connected:
            if self.motor_running:
                self.lbl_status.configure(text="Durum: Motor Dönürken Kaydedilemez!", text_color="red")
                return
            self.lbl_status.configure(text="Durum: Flaşa Yazılıyor...", text_color="orange")
            self.send_reliable(0x70, retries=3)

    def heartbeat_worker(self):
        while True:
            time.sleep(0.3) 
            if self.is_connected:
                self.send_reliable(0x01, b"", retries=1) 

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
                        
                        elif line.startswith("<PRM,"): # Cihazdan Parametre Okuma Yanıtı
                            parts = line.strip("<>").split(",")
                            if len(parts) >= 7:
                                kp = float(parts[1])
                                ki = float(parts[2])
                                osc_time = float(parts[3])
                                osc_deg = float(parts[4])
                                osc_rpm = float(parts[5])
                                osc_accel = float(parts[6])
                                # Arayüzü güvenle güncelle (Thread-Safe)
                                self.after(0, self.update_ui_from_params, kp, ki, osc_time, osc_deg, osc_rpm, osc_accel)

                        elif line.startswith("<DBG: PARAMS_SAVED_FLASH"):
                            self.after(0, lambda: self.lbl_status.configure(text="Durum: Ayarlar Kalıcı Olarak Kaydedildi!", text_color="#f1c40f"))

                        elif line.startswith("<DBG: CMD_") or line.startswith("<DBG: HB_OK"):
                            self.ack_event.set()
                            if not line.startswith("<DBG: HB_OK"):
                                self.lbl_status.configure(text="Durum: Komut Başarılı", text_color="green")
                            
                        elif line.startswith("<DBG: WATCHDOG"):
                            self.lbl_status.configure(text="Durum: WATCHDOG DEVREDE!", text_color="red")
            except:
                pass

if __name__ == "__main__":
    app = ErtipCrownApp()
    app.mainloop()