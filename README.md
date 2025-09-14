# Proyek Robot Visual Servoing dengan ROS2 & Gazebo

![ROS Humble](https://img.shields.io/badge/ROS-Humble-blue?style=for-the-badge&logo=ros)
![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic-orange?style=for-the-badge)
![Python](https://img.shields.io/badge/Python-3.10-blue?style=for-the-badge&logo=python)
![OpenCV](https://img.shields.io/badge/OpenCV-4.5-blue?style=for-the-badge&logo=opencv)

Repositori ini berisi implementasi lengkap dari robot mobile otonom yang melakukan tugas *visual servoing*—menggunakan input visual dari kamera untuk mengontrol gerakannya. Robot disimulasikan di Gazebo Classic dan dioperasikan menggunakan ROS 2 Humble.

Tujuan utama robot adalah untuk secara otomatis mendeteksi objek berwarna hijau, mendekatinya, dan berhenti pada jarak yang telah ditentukan.

## Demo Visual
[Screencast from 09-15-2025 12:00:37 AM.webm](https://github.com/user-attachments/assets/3e796a59-1cac-4682-87be-47df6a4c538a)
## Konsep Inti & Filosofi Desain

### Visual Servoing
Visual Servoing adalah kelas teknik kontrol robot yang menggunakan umpan balik (feedback) dari sensor visual (seperti kamera) untuk mengontrol gerakan robot. Alih-alih mengandalkan sensor odometri atau GPS yang bisa tidak akurat, sistem ini secara dinamis menghitung "error" antara posisi target saat ini di dalam gambar dan posisi yang diinginkan, lalu menggunakan error tersebut untuk menggerakkan robot.

### Kontrol PID Ganda (Decoupled Control)
Sistem kontrol inti dari proyek ini menggunakan **dua kontroler PD (Proportional-Derivative) independen**. Pendekatan ini memecah masalah kompleks "mengemudi ke suatu titik" menjadi dua masalah yang lebih sederhana:
1.  **Kontrol Linear (`pid_y`):** Kontroler ini hanya bertanggung jawab atas kecepatan **maju/mundur**. Inputnya adalah "error jarak", yang dalam proyek ini diwakili oleh *ukuran area* objek yang terlihat. Semakin kecil areanya, semakin jauh targetnya, dan semakin besar perintah maju yang diberikan.
2.  **Kontrol Angular (`pid_x`):** Kontroler ini hanya bertanggung jawab atas **kemudi (belok kiri/kanan)**. Inputnya adalah "error posisi horizontal", yaitu selisih piksel antara pusat gambar dan pusat objek.

Dengan memisahkan kedua tugas ini, proses *tuning* menjadi lebih intuitif dan sistem lebih stabil.

### Logika Perilaku (Behavioral Logic)
Robot ini beroperasi dalam dua status utama:
-   **Pelacakan (Tracking):** Saat objek terlihat, kedua kontroler PID aktif, dan robot secara dinamis menyesuaikan kecepatan dan kemudi untuk mendekati target.
-   **Pencarian (Searching):** Jika objek hilang dari pandangan (misalnya, karena terhalang atau robot overshoot), sistem akan masuk ke mode *failsafe*. Robot akan berhenti maju dan berputar di tempat dengan kecepatan yang ditentukan untuk memindai lingkungan sampai objek ditemukan kembali.

## Arsitektur Sistem & Aliran Data

Proyek ini mengikuti arsitektur berbasis node standar ROS 2. Aliran data utama dari sensor ke aktuator adalah sebagai berikut:



| Publisher | Topic | Tipe Pesan | Subscriber | Keterangan |
| :--- | :--- | :--- | :--- | :--- |
| **Gazebo (Kamera)** | `/camera_sensor/image_raw`| `sensor_msgs/Image` | `object_detection` | Feed video mentah dari robot. |
| `object_detection` | `/object_error_x` | `std_msgs/Float64` | `pid_x` | Error posisi horizontal (dalam piksel). |
| `object_detection`| `/object_error_y` | `std_msgs/Float64` | `pid_y` | Error jarak (berdasarkan area kontur). |
| `pid_x` | `/angular_correction` | `std_msgs/Float64` | `movement_control` | Perintah koreksi kemudi (rad/s). |
| `pid_y` | `/linear_correction` | `std_msgs/Float64` | `movement_control` | Perintah koreksi kecepatan (m/s). |
| `movement_control` | `/cmd_vel` | `geometry_msgs/Twist`| **Gazebo (Diff Drive)** | Perintah kecepatan akhir untuk roda. |

## Struktur Workspace

Workspace ini diorganisir menjadi tiga paket ROS 2 yang berbeda untuk modularitas:

robot_ws/
└── src/
├── robot_description/  # Definisi robot (URDF), model, dan file launch utama.
├── robot_control/      # Semua logika inti (deteksi, PID, kontrol gerakan).
└── robot_navigation/   # Placeholder untuk algoritma navigasi masa depan (A*).


## Penyiapan dan Instalasi

### Prasyarat

-   Ubuntu 22.04 LTS
-   ROS 2 Humble Hawksbill (Instalasi `ros-humble-desktop` direkomendasikan)
-   Gazebo Classic (terinstal bersama `ros-humble-desktop`)
-   `colcon` (alat build standar ROS 2)

### Langkah-langkah

1.  **Clone Repositori ke Folder `src` Workspace Anda**
    ```bash
    mkdir -p ~/robot_ws/src
    cd ~/robot_ws/src
    git clone [https://github.com/Camn0/ros2-visual-servoing-pid.git](https://github.com/Camn0/ros2-visual-servoing-pid.git) .
    ```

2.  **Instal Dependensi** (Meskipun proyek ini menggunakan paket standar, ini adalah praktik yang baik)
    ```bash
    cd ~/robot_ws
    rosdep install -i --from-path src --rosdistro humble -y
    ```

3.  **Build Workspace**
    ```bash
    cd ~/robot_ws
    colcon build --symlink-install
    ```

## Cara Menjalankan

Setelah workspace berhasil di-build, Anda dapat meluncurkan seluruh simulasi dan tumpukan kontrol dengan satu perintah.

1.  **Buka Terminal Baru dan Sumber (Source) Workspace Anda:**
    ```bash
    source ~/robot_ws/install/setup.bash
    ```
2.  **Luncurkan File Launch Utama:**
    ```bash
    ros2 launch robot_description display.launch.py
    ```
    Perintah ini akan:
    -   Membuka jendela simulator Gazebo.
    -   Memunculkan (spawn) robot di titik (0,0).
    -   Memunculkan plat target berwarna hijau di depannya.
    -   Memulai semua node Python yang diperlukan untuk deteksi dan kontrol.

## Panduan Konfigurasi & Tuning Perilaku

Perilaku robot sangat sensitif terhadap beberapa parameter kunci. Menyesuaikannya adalah bagian penting dari proses. **Selalu jalankan `colcon build` setelah mengubah file Python.**

### 1. Jarak Berhenti (Parameter Paling Kritis)

Ini adalah perbaikan paling penting yang ditemukan selama pengembangan. Jika robot berperilaku tidak menentu saat sangat dekat dengan target, masalahnya ada di sini.

-   **File:** `src/robot_control/src/object_detection.py`
-   **Parameter:** `target_area_setpoint`
-   **Penjelasan:** Parameter ini adalah "tujuan" dari kontroler kecepatan (`pid_y`). Ini adalah ukuran (dalam piksel persegi) dari kontur hijau yang Anda inginkan saat robot berhenti. Masalah muncul ketika nilai ini secara fisik tidak dapat dicapai—yaitu, ketika plat hijau mulai terpotong oleh tepi kamera sebelum areanya bisa mencapai setpoint. Ketika ini terjadi, area yang terdeteksi tiba-tiba menyusut, menyebabkan PID berpikir target menjauh dan memerintahkan **akselerasi**, yang menyebabkan *overshoot*.
    -   **Jika robot melaju kencang/overshoot saat mendekat:** Nilai ini **terlalu besar**. **Turunkan nilainya** (misalnya dari `50000.0` ke `35000.0` atau `10000.0`).
    -   **Jika robot berhenti terlalu jauh:** Anda bisa sedikit **menaikkan nilainya**.

### 2. Kecepatan & Stabilitas (Tuning PID)

-   **File:** `src/robot_control/src/pid_x.py` (Kemudi) & `pid_y.py` (Kecepatan Maju)
-   **Parameter:** `self.kp`, `self.kd`

| Parameter | Efek | Cara Tuning |
| :--- | :--- | :--- |
| **`pid_y.py` -> `kp`** | **"Gas" Utama:** Mengontrol kecepatan maju maksimum. | Naikkan untuk membuat robot lebih cepat. Turunkan jika menjadi tidak stabil atau overshoot. |
| **`pid_y.py` -> `kd`** | **"Rem" Utama:** Meredam osilasi maju-mundur dan mencegah overshoot jarak. | Naikkan secara signifikan jika robot overshoot. Turunkan jika robot terasa terlalu "berat" atau lambat untuk mulai bergerak. |
| **`pid_x.py` -> `kp`** | **"Agresivitas Kemudi":** Mengontrol seberapa cepat robot berbelok untuk menghadap target. | Naikkan untuk belokan yang lebih "snappy" dan responsif. |
| **`pid_x.py` -> `kd`** | **"Peredam Kemudi":** Mencegah getaran (wobble) dari sisi ke sisi saat robot mencoba mengunci target. | Naikkan jika robot bergetar saat menghadap lurus ke target. |


### 3. Kecepatan Pencarian

-   **File:** `src/robot_control/src/movement_control.py` atau `src/robot_control/launch/control.launch.py`
-   **Parameter:** `search_angular_velocity`
-   **Penjelasan:** Mengontrol seberapa cepat robot berputar di tempat (dalam rad/s) saat mencari plat hijau yang hilang. Nilai ini dapat diubah langsung di file `movement_control.py` atau diganti (override) di file `control.launch.py` untuk penyesuaian yang lebih mudah.

## Troubleshooting Umum

-   **Gazebo Gagal Memulai (`Address already in use`):** Ini berarti ada proses Gazebo "zombie" yang berjalan di latar belakang. Matikan paksa semua proses:
    ```bash
    killall -9 gzserver && killall -9 gzclient
    ```
-   **`rqt_image_view: command not found`:** Alat visualisasi kamera tidak terinstal. Instal dengan:
    ```bash
    sudo apt update
    sudo apt install ros-humble-rqt-image-view
    ```

## Rencana Pengembangan di Masa Depan

-   **Integrasi Navigasi:** Mengintegrasikan paket `robot_navigation` dengan data dari Lidar 2D untuk melakukan pathfinding A* atau SLAM di sekitar rintangan.
-   **Penghindaran Rintangan (Obstacle Avoidance):** Menggunakan data Lidar untuk secara aktif menghindari rintangan saat mendekati target, bahkan tanpa peta.
-   **Tuning PID Dinamis:** Mengimplementasikan node yang memungkinkan gain PID diubah secara real-time menggunakan parameter ROS2 untuk tuning yang lebih cepat dan interaktif.

## Lisensi

Proyek ini dilisensikan di bawah Lisensi MIT. Lihat file `LICENSE` untuk detailnya.
