# Proyek Robot Visual Servoing dengan ROS2 dan Gazebo

![ROS Humble](https://img.shields.io/badge/ROS-Humble-blue?style=for-the-badge&logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Simulator-orange?style=for-the-badge)
![Python](https://img.shields.io/badge/Python-3.10-blue?style=for-the-badge&logo=python)

Ini adalah proyek tugas besar yang mengimplementasikan robot mobile sederhana di dalam simulator Gazebo. Robot ini menggunakan kamera dan logika kontrol PID ganda untuk secara otonom mendeteksi, mendekati, dan berhenti di depan objek berwarna hijau.

## Demo Visual



*[Screencast from 09-15-2025 12:00:37 AM.webm](https://github.com/user-attachments/assets/3e796a59-1cac-4682-87be-47df6a4c538a)*

## Fitur Utama

- **Simulasi di Gazebo:** Lingkungan simulasi lengkap dengan model robot (URDF/XACRO) yang dilengkapi sensor seperti Kamera, Lidar 2D, dan IMU.
- **Deteksi Objek (OpenCV):** Sebuah node Python yang berlangganan ke feed kamera, mengisolasi objek berwarna hijau menggunakan thresholding HSV, dan menghitung error posisinya.
- **Kontrol PID Ganda:** Dua kontroler PID terpisah menangani:
    1.  **Gerakan Linear (Maju/Mundur):** Mengontrol kecepatan robot berdasarkan ukuran objek yang terlihat (area kontur) untuk berhenti pada jarak yang diinginkan.
    2.  **Gerakan Angular (Belok Kiri/Kanan):** Mengontrol kemudi robot untuk menjaga objek tetap di tengah bidang pandang kamera.
- **Logika Perilaku:**
    - **Pelacakan (Tracking):** Saat objek terlihat, robot akan aktif mendekatinya.
    - **Pencarian (Searching):** Jika objek hilang dari pandangan, robot akan berhenti maju dan berputar di tempat untuk mencarinya kembali.
- **Arsitektur Modular ROS2:** Proyek diorganisir menjadi tiga paket ROS2 yang berbeda untuk deskripsi, kontrol, dan navigasi.

## Arsitektur Sistem

Aliran data di antara node-node ROS2 adalah sebagai berikut:

| Publisher | Topic | Subscriber | Keterangan |
| :--- | :--- | :--- | :--- |
| **Gazebo (Plugin Kamera)** | `/camera_sensor/image_raw` | `object_detection` | Feed video mentah dari robot. |
| **`object_detection`** | `/object_error_x` | `pid_x` | Error posisi horizontal (piksel). |
| **`object_detection`**| `/object_error_y` | `pid_y` | Error jarak (area kontur piksel). |
| **`pid_x`** | `/angular_correction` | `movement_control` | Perintah koreksi kemudi (rad/s). |
| **`pid_y`** | `/linear_correction` | `movement_control` | Perintah koreksi kecepatan (m/s). |
| **`movement_control`** | `/cmd_vel` | **Gazebo (Plugin Diff Drive)** | Perintah kecepatan akhir untuk roda. |

## Struktur Paket

-   `robot_ws/`
    -   `src/`
        -   **`robot_description/`**: Berisi semua file yang mendefinisikan robot (URDF/XACRO), model 3D, dan file launch utama untuk memulai simulasi.
        -   **`robot_control/`**: Berisi semua logika inti—node deteksi objek, node PID, dan node kontrol gerakan.
        -   **`robot_navigation/`**: Paket placeholder yang berisi algoritma pathfinding A* (saat ini tidak terintegrasi) untuk pengembangan di masa depan.

## Penyiapan dan Instalasi

### Prasyarat

-   Ubuntu 22.04
-   ROS 2 Humble Hawksbill
-   Gazebo (biasanya terinstal bersama `ros-humble-desktop`)
-   Colcon (alat build ROS2)

### Langkah-langkah

1.  **Clone Repositori**
    ```bash
    cd ~/
    git clone [https://github.com/NAMA_ANDA/NAMA_REPO_ANDA.git](https://github.com/NAMA_ANDA/NAMA_REPO_ANDA.git) robot_ws/src
    ```

2.  **Instal Dependensi** (Proyek ini menggunakan dependensi standar ROS, tetapi ini adalah praktik yang baik)
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

Setelah workspace berhasil di-build, jalankan simulasi lengkap dengan satu perintah launch:

1.  **Sumber (Source) Workspace Anda:**
    ```bash
    source ~/robot_ws/install/setup.bash
    ```
2.  **Luncurkan File Launch Utama:**
    ```bash
    ros2 launch robot_description display.launch.py
    ```
    Ini akan membuka jendela Gazebo, memunculkan robot dan plat hijau, dan memulai semua node kontrol secara otomatis.

## Konfigurasi & Tuning

Perilaku robot dapat disesuaikan dengan mengubah beberapa parameter kunci dalam kode. Setelah mengubah parameter apa pun, jangan lupa untuk membangun ulang workspace dengan `colcon build`.

### 1. Menyesuaikan Jarak Berhenti (Perbaikan Paling Penting)

Ini adalah parameter yang paling krusial untuk stabilitas. Jika robot melaju kencang saat mendekat, itu karena setpoint ini tidak dapat dicapai.

-   **File:** `src/robot_control/src/object_detection.py`
-   **Parameter:** `target_area_setpoint`
-   **Penjelasan:** Nilai ini adalah "tujuan" dari kontroler PID kecepatan. Ini adalah ukuran (dalam piksel persegi) dari kontur hijau yang Anda inginkan saat robot berhenti.
    -   **Jika robot overshoot atau melaju kencang saat mendekat:** Nilai ini **terlalu besar**. Plat hijau sudah terpotong oleh tepi kamera sebelum areanya bisa mencapai setpoint ini. **Turunkan nilainya** (misalnya dari `50000.0` ke `35000.0` atau `10000.0`).
    -   **Jika robot berhenti terlalu jauh:** **Naikkan nilainya** sedikit.

### 2. Menyesuaikan Kecepatan & Stabilitas (File PID)

-   **File:** `src/robot_control/src/pid_x.py` (Kemudi) & `pid_y.py` (Kecepatan Maju)
-   **Parameter:** `self.kp`, `self.kd`

    -   **`kp` (Proportional Gain):** Anggap ini sebagai "pedal gas". Nilai yang lebih tinggi membuat robot merespons lebih cepat dan lebih agresif. Jika terlalu tinggi, robot akan menjadi tidak stabil dan berosilasi (gemetar atau maju-mundur).
    -   **`kd` (Derivative Gain):** Anggap ini sebagai "rem" atau "peredam kejut". Nilai yang lebih tinggi akan meredam osilasi dan mencegah robot *overshoot* (kebablasan). Jika terlalu tinggi, respons robot akan terasa lambat atau "berat".

### 3. Menyesuaikan Kecepatan Pencarian

-   **File:** `src/robot_control/src/movement_control.py`
-   **Parameter:** `search_angular_velocity`
-   **Penjelasan:** Mengontrol seberapa cepat robot berputar di tempat saat mencari plat hijau yang hilang. Naikkan nilainya untuk pencarian yang lebih cepat.
