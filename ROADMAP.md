# Ertip Crown V1.3 FUE Mikromotor Kontrolcüsü - Geliştirme Yol Haritası

**Proje:** Ertip Crown V1.3
**Geliştirici:** Anıl Akman
**Donanım:** STM32F103C8T6 (ARM Cortex-M3)
**Güncel Durum:** Faz 1 Tamamlandı. Faz 4 Testleri yapılıyor. (5 Mart 2026)

---

## 🎯 Proje Hedefleri
Mevcut çalışan donanım ve yazılım mimarisini koruyarak sistemi aşağıdaki standartlara yükseltmek:
1. Kesintisiz ve elektriksel gürültüden (EMI) etkilenmeyen güvenli bir haberleşme protokolü kurmak. *(Büyük oranda tamamlandı)*
2. Cihaz parametrelerinin (PID, İvme, Maksimum Hız vb.) EEPROM üzerinde kalıcı olarak saklanmasını sağlamak.
3. Donanımsal FPU eksikliğinden kaynaklanan işlemci yükünü optimize etmek.
4. "Data Race" (Veri Yarışı) ve değişken taşması (Overflow) gibi gizli hataları kalıcı olarak çözmek.

---

## 🚀 Geliştirme Fazları ve Güncel İlerleme (DURUM RAPORU)

### Faz 1: Yeni Nesil ve Güvenli Haberleşme Altyapısı (✅ TAMAMLANDI)
Cihazın dış dünya ile iletişimini endüstriyel standartlara taşıdık.
- [x] **1.1. Paket Mimarisinin Tasarımı:** ASCII tabanlı string haberleşmesi tamamen silindi. Yerine `0xAA 0x55` başlık senkronizasyonlu, değişken uzunluklu ve **CRC-16 Modbus** korumalı Endüstriyel Binary sisteme geçildi.
- [x] **1.2. UART IDLE Line Interrupt Kurulumu:** *Değişiklik:* CubeIDE'nin DMA konfigürasyon bug'ları nedeniyle DMA yerine, %100 garantili olan `ReceiveToIdle_IT` (Hat boşta kesmesi) mimarisine geçildi. Cihaz artık byte-byte değil, paket bazlı uyanıyor.
- [x] **1.3. Parser ve Dispatcher Yazılması:** Gelen paketin CRC onayı yapılıp, `Process_Binary_Packet` ile motor komutlarına (0x10 SET_RPM, 0x20 STOP) dönüştürülmesi başarıyla sağlandı.
- [x] **1.4. (Ekstra) UART Hata Kurtarıcı:** Bağlantı kopmalarına ve gürültülere karşı sistemi yeniden başlatan `HAL_UART_ErrorCallback` eklendi.

### Faz 2: Kalıcı Bellek (EEPROM) ve Parametre Yönetimi (⏳ BEKLİYOR)
- [ ] **2.1. Parametre Haritasının Çıkarılması:** Değişmesi muhtemel tüm sistem değişkenlerinin (PID katsayıları, hız ve ivme limitleri vb.) `DeviceParams_t` yapısında toplanması.
- [ ] **2.2. I2C EEPROM Sürücüsünün Entegrasyonu:** Kart üzerindeki EEPROM entegresi için sayfa bazlı güvenli okuma/yazma donanım katmanı yazılması.
- [ ] **2.3. Boot ve Save Mekanizması:** Açılışta RAM'e çekme ve haberleşme ile kalıcı kaydetme işlemleri.

### Faz 3: Kontrol Döngüsü (Control Loop) Optimizasyonu ve Güvenlik (🔥 AKTİF ÇALIŞMA ALANI)
Şu an testlerde karşılaştığımız sorunların çözüleceği ana faz burasıdır.
- [ ] **3.0. (KRİTİK) Runaway Motor ve Komut Kaçırma Bug'ı:** Cihazın dur komutunu algılayamayıp kontrolsüz hızlanması sorununun (Integrator Windup / UART kilitlenmesi) teşhis edilip çözülmesi.
- [ ] **3.1. TIM2 Performans Ölçümü ve Optimizasyonu:** 5ms'lik kontrol döngüsünün osiloskop ile ölçülmesi, ağır `float` (`sqrtf`) işlemlerinin optimize edilmesi.
- [ ] **3.2. Data Race (Veri Yarışı) Koruması:** Kesme (ISR) ile ana döngü çakışmalarını önlemek için kritik blok korumaları (`__disable_irq()`).
- [x] **3.3. StallSupervisor Revizyonu:** Sıkışma korumasının sadece Pozisyon/Osilasyon modunda çalışacak şekilde izole edilmesi sağlandı. (Motorun Continuous modda kalkış yapamaması sorunu çözüldü).
- [ ] **3.4. Enkoder Taşma (Overflow) Koruması:** Sürekli modda değişkenin taşmasını engelleyecek güvenlik sıfırlaması.

### Faz 4: PC Test Arayüzü ve Entegrasyon (🚧 DEVAM EDİYOR)
- [x] **4.1. Test Script'inin Hazırlanması:** PC tarafında çift yönlü haberleşen, CRC hesaplayan ve telemetri okuyan `motor_test.py` aracı yazıldı.
- [~] **4.2. Stres Testleri:** Haberleşme yoğunluğu ve PID stabilitesi şu an aktif olarak test ediliyor.