# Robot Arm 5-DOF — Forward & Inverse Kinematics

Proyek edukasi robot arm lima derajat kebebasan (5-DOF) untuk menunjukkan pemahaman **Forward Kinematics (FK)** dan **Inverse Kinematics (IK)** melalui simulasi Python, dengan firmware ESP32 sebagai tahap pengembangan perangkat keras berikutnya.

> **Status:** simulator Python dan model URDF disediakan sebagai basis eksperimen. Firmware ESP32 adalah starter untuk kontrol sudut servo manual melalui web server lokal; integrasi perhitungan FK/IK ke firmware belum diimplementasikan.

## Tujuan 

- Memahami hubungan antara sudut joint dan posisi ujung robot melalui FK.
- Memahami pencarian sudut joint dari koordinat target melalui IK numerik.
- Mengamati pengaruh sumbu rotasi, panjang link, batas joint, dan konfigurasi robot terhadap ruang kerja.
- Membandingkan hasil FK dengan posisi aktual yang dihitung kembali setelah IK.
- Mendokumentasikan asumsi model, pengujian, serta keterbatasan solver.

## Struktur repository


## Arsitektur robot

| Joint | Nama URDF | Fungsi | Rentang model |
|---|---|---|---|
| 1 | `base_yaw` | Memutar basis pada sumbu vertikal | -90° sampai +90° relatif ke posisi nol |
| 2 | `shoulder_pitch` | Mengangkat/menurunkan lengan pertama | -80° sampai +80° relatif ke posisi nol |
| 3 | `elbow_pitch` | Menekuk lengan kedua | -90° sampai +90° relatif ke posisi nol |
| 4 | `wrist_pitch` | Mengubah kemiringan link ujung | -80° sampai +80° relatif ke posisi nol |
| 5 | `end_yaw` | Memutar link terakhir pada sumbu vertikal | -90° sampai +90° relatif ke posisi nol |

Input sudut servo di antarmuka Python menggunakan 0–180°, dengan 90° dipetakan ke sudut joint 0 radian. Batas aktual tetap mengikuti batas joint di URDF; karena itu tidak semua sudut 0–180° selalu diterima oleh setiap joint.

Model URDF menggunakan geometri primitif sehingga bisa divisualisasikan tanpa file mesh eksternal. Dimensi adalah model awal untuk pembelajaran, bukan pengukuran mekanik final.

## 1. Menjalankan simulasi Python

### Prasyarat

- Python 3.10+ disarankan.
- Paket: NumPy, Robotics Toolbox for Python, SpatialMath, dan Swift Simulator.

Instalasi:

```bash
python -m venv .venv
source .venv/bin/activate        # Linux/macOS
# .venv\Scripts\activate         # Windows PowerShell

python -m pip install --upgrade pip
python -m pip install numpy roboticstoolbox-python spatialmath-python swift-sim
```

Jalankan dari folder repository:

```bash
python simulation/simulator.py
```

Swift akan membuka visualisasi pada browser. Jika instalasi library atau browser backend gagal, periksa pesan error dari terminal dan versi Python yang digunakan.

### Forward Kinematics (FK)

Pilih menu `1`, lalu masukkan sudut Base, Shoulder, Elbow, Wrist, dan End Joint.

Program mengubah sudut input menjadi radian, mengevaluasi transformasi ujung dengan `arm.fkine(q)`, kemudian menampilkan koordinat X, Y, dan Z dalam milimeter.

Secara konsep:

\[
{}^{0}T_{n} = {}^{0}T_{1}(q_1)\,{}^{1}T_{2}(q_2)\cdots{}^{n-1}T_n(q_n)
\]

Transformasi total adalah perkalian transformasi setiap joint dan link sepanjang rantai kinematik.

### Inverse Kinematics (IK)

Pilih menu `2`, lalu masukkan target X, Y, Z dalam milimeter. Solver Levenberg–Marquardt mencari konfigurasi joint yang membawa `tool_tip` ke posisi target.

IK pada versi ini **mengunci posisi titik ujung, bukan orientasi lengkap tool**. Karena ada lima joint untuk target posisi 3D, bisa terdapat beberapa solusi. Solver dapat gagal jika target di luar ruang kerja, dekat singularitas, atau tidak kompatibel dengan batas joint.

Program menghitung ulang FK pada solusi IK untuk mengukur error posisi. Keberhasilan solver numerik tidak menjamin semua konfigurasi mekanik nyata aman.

## 2. Firmware ESP32 (opsional, untuk tahap hardware)

Folder `firmware/` adalah jalur pengembangan berikutnya. Firmware starter menyediakan:
- ESP32 membuat Wi-Fi Access Point `RobotArm-5DOF`.
- Halaman web lokal dengan slider sudut untuk lima servo.
- PCA9685 mengeluarkan sinyal servo pada channel 0–4.
- OLED SSD1306 menampilkan IP AP dan sudut tiap joint.

Firmware ini **belum menjalankan FK atau IK**; pembuktian konsep FK/IK utama dilakukan melalui simulator Python. Rencana lanjutan adalah menyamakan model joint, offset sudut, dan batas servo antara firmware dan simulator.

### Build dengan PlatformIO

1. Buka folder `firmware/` di VS Code dengan ekstensi PlatformIO.
2. Build dan upload ke board ESP32 yang sesuai.
3. Buka Serial Monitor pada 115200 baud.
4. Sambungkan komputer/ponsel ke Wi-Fi `RobotArm-5DOF` dengan kata sandi yang didefinisikan di `main.cpp`.
5. Buka alamat IP yang tercetak di Serial Monitor (umumnya `192.168.4.1`).

### Peringatan hardware

- Jangan menghubungkan servo langsung ke pin 3.3 V ESP32.
- Gunakan catu daya servo eksternal yang sesuai dan satukan referensi GND dengan sistem kontrol.
- Nilai pulse minimum/maksimum pada firmware hanyalah nilai awal. Kalibrasikan untuk setiap servo dan mekanisme sebelum memasang link.
- Lepaskan link atau beban saat pengujian pertama, sediakan tombol pemutus daya, dan jangan memaksa servo ke batas mekanik.
- Periksa alamat I²C OLED dan PCA9685 pada modul yang benar-benar digunakan.

## 3. Rencana validasi untuk laporan

Gunakan tabel eksperimen berikut untuk membuktikan pemahaman, bukan hanya menunjukkan visualisasi.

| Uji | Prosedur | Yang dicatat |
|---|---|---|
| FK-1 | Atur semua joint ke 90° | Posisi awal X, Y, Z |
| FK-2 | Ubah satu joint pada satu waktu | Perubahan koordinat ujung |
| IK-1 | Tentukan target yang dapat dijangkau | Sudut joint hasil solver |
| IK-2 | Masukkan hasil IK kembali ke FK | Error posisi target |
| IK-3 | Uji target di luar ruang kerja | Pesan kegagalan/penanganan |
| Limit | Coba sudut di luar batas joint | Validasi input |
| Repeatability | Ulangi target yang sama dari konfigurasi awal berbeda | Perbedaan solusi dan error |

Sertakan screenshot Swift, input yang digunakan, output koordinat, sudut joint hasil IK, dan error posisi. Jelaskan juga mengapa IK bisa memiliki banyak solusi dan mengapa posisi target tidak menentukan orientasi tool secara unik.

## Asumsi dan keterbatasan

- Model kinematik ideal; tidak menghitung kelenturan, backlash, gravitasi, torsi, atau tabrakan antar-link.
- Panjang link dan offset URDF merupakan parameter awal dan harus disesuaikan dengan desain mekanik sebenarnya.
- Pemetaan sudut servo `servo_deg - 90°` adalah asumsi nol-servo; arah dan offset aktual dapat berbeda pada perangkat nyata.
- Model IK mengutamakan posisi titik ujung, bukan orientasi tool.
- Firmware adalah starter eksperimen dan harus diuji serta dikalibrasi sebelum digunakan pada perangkat fisik.

## Teknologi

- Python
- NumPy
- Robotics Toolbox for Python
- SpatialMath
- Swift Simulator
- URDF
- PlatformIO / Arduino framework (firmware opsional)
- ESP32, PCA9685, OLED SSD1306 (hardware opsional)


