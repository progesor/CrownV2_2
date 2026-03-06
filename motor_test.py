import serial
import struct
import time
import threading

# --- AYARLAR ---
COM_PORT = 'COM7'  # BURAYI KENDİ PORTUNA GÖRE DEĞİŞTİR
BAUD_RATE = 115200

def calculate_crc16(data: bytes) -> int:
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

def build_packet(cmd: int, payload: bytes = b"") -> bytes:
    length = len(payload)
    crc_data = struct.pack('<BB', length, cmd) + payload
    crc = calculate_crc16(crc_data)
    crc_l = crc & 0xFF
    crc_h = (crc >> 8) & 0xFF
    packet = struct.pack('<BB', 0xAA, 0x55) + crc_data + struct.pack('<BB', crc_l, crc_h)
    return packet

def send_set_rpm(ser, rpm: float):
    payload = struct.pack('<f', float(rpm)) 
    packet = build_packet(0x10, payload)
    ser.write(packet)
    print(f"\n[BİLGİSAYAR] GÖNDERİLDİ - SET RPM: {rpm}")

def send_stop(ser):
    packet = build_packet(0x20)
    ser.write(packet)
    print(f"\n[BİLGİSAYAR] GÖNDERİLDİ - STOP")

# Arka planda STM32'den gelen verileri okuyan fonksiyon
def read_from_stm32(ser):
    while ser.is_open:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"[STM32-CEVAP] {line}")
        except:
            break

# --- ANA TEST DÖNGÜSÜ ---
if __name__ == "__main__":
    try:
        print(f"{COM_PORT} Portuna Bağlanılıyor...")
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(1)
        print("Bağlantı Başarılı! Telemetri verileri bekleniyor...\n")

        # Dinleme thread'ini başlat
        reader_thread = threading.Thread(target=read_from_stm32, args=(ser,), daemon=True)
        reader_thread.start()

        time.sleep(2) # İlk telemetrileri görmek için biraz bekle

        # Senaryo 1: Motoru 1000 RPM ile döndür
        send_set_rpm(ser, 14000.0)
        time.sleep(4) 
        
        # Senaryo 2: Motoru Durdur
        send_stop(ser)
        time.sleep(2)

        ser.close()
        print("\nTest Tamamlandı.")
            
    except serial.SerialException as e:
        print(f"Seri Port Hatası: {e}")