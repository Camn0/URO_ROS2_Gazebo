Proyek Robot Visual Servoing dengan ROS2 dan Gazebo

Ini adalah proyek tugas besar yang mengimplementasikan robot mobile sederhana di dalam simulator Gazebo. Robot ini menggunakan kamera dan logika kontrol PID ganda untuk secara otonom mendeteksi, mendekati, dan berhenti di depan objek berwarna hijau.
Demo Visual

Fitur Utama

    Simulasi di Gazebo: Lingkungan simulasi lengkap dengan model robot (URDF/XACRO) yang dilengkapi sensor seperti Kamera, Lidar 2D, dan IMU.

    Deteksi Objek (OpenCV): Sebuah node Python yang berlangganan ke feed kamera, mengisolasi objek berwarna hijau menggunakan thresholding HSV, dan menghitung error posisinya.

    Kontrol PID Ganda: Dua kontroler PID terpisah menangani:

        Gerakan Linear (Maju/Mundur): Mengontrol kecepatan robot berdasarkan ukuran objek yang terlihat (area kontur) untuk berhenti pada jarak yang diinginkan.

        Gerakan Angular (Belok Kiri/Kanan): Mengontrol kemudi robot untuk menjaga objek tetap di tengah bidang pandang kamera.

    Logika Perilaku:

        Pelacakan (Tracking): Saat objek terlihat, robot akan aktif mendekatinya.

        Pencarian (Searching): Jika objek hilang dari pandangan, robot akan berhenti maju dan berputar di tempat untuk mencarinya kembali.

    Arsitektur Modular ROS2: Proyek diorganisir menjadi tiga paket ROS2 yang berbeda untuk deskripsi, kontrol, dan navigasi.

Arsitektur Sistem

Aliran data di antara node-node ROS2 adalah sebagai berikut:

    Gazebo (Plugin Kamera) mempublikasikan gambar mentah ke topic /camera_sensor/image_raw.

    Node object_detection berlangganan ke /camera_sensor/image_raw.

        Ia menghitung dua jenis error:

            Error Angular (posisi horizontal) dipublikasikan ke /object_error_x.

            Error Jarak (ukuran area) dipublikasikan ke /object_error_y.

    Node pid_x berlangganan ke /object_error_x dan mempublikasikan koreksi kemudi ke /angular_correction.

    Node pid_y berlangganan ke /object_error_y dan mempublikasikan koreksi kecepatan maju ke /linear_correction.

    Node movement_control berlangganan ke /angular_correction dan /linear_correction.

        Ia menggabungkan kedua sinyal koreksi menjadi satu pesan Twist dan mempublikasikannya ke /cmd_vel.

    Gazebo (Plugin Differential Drive) berlangganan ke /cmd_vel untuk menggerakkan roda robot.

Struktur Paket

    robot_ws/

        src/

            robot_description/: Berisi semua file yang mendefinisikan robot (URDF/XACRO), model 3D, dan file launch utama untuk memulai simulasi.

            robot_control/: Berisi semua logika inti—node deteksi objek, node PID, dan node kontrol gerakan.

            robot_navigation/: Paket placeholder yang berisi algoritma pathfinding A* (saat ini tidak terintegrasi) untuk pengembangan di masa depan.

Penyiapan dan Instalasi
Prasyarat

    Ubuntu 22.04

    ROS 2 Humble Hawksbill

    Gazebo (biasanya terinstal bersama ros-humble-desktop)

    Colcon (alat build ROS2)

Langkah-langkah

    Clone Repositori

    cd ~/
    git clone [https://github.com/NAMA_ANDA/NAMA_REPO_ANDA.git](https://github.com/NAMA_ANDA/NAMA_REPO_ANDA.git) robot_ws/src

    Instal Dependensi (Proyek ini menggunakan dependensi standar ROS, tetapi ini adalah praktik yang baik)

    cd ~/robot_ws
    rosdep install -i --from-path src --rosdistro humble -y

    Build Workspace

    cd ~/robot_ws
    colcon build --symlink-install

Cara Menjalankan

Setelah workspace berhasil di-build, jalankan simulasi lengkap dengan satu perintah launch:

    Sumber (Source) Workspace Anda:

    source ~/robot_ws/install/setup.bash

    Luncurkan File Launch Utama:

    ros2 launch robot_description display.launch.py

    Ini akan membuka jendela Gazebo, memunculkan robot dan plat hijau, dan memulai semua node kontrol secara otomatis.

Konfigurasi & Tuning

Perilaku robot dapat disesuaikan dengan mengubah beberapa parameter kunci dalam kode. Setelah mengubah parameter apa pun, jangan lupa untuk membangun ulang workspace dengan colcon build.
1. Menyesuaikan Kecepatan & Stabilitas (File PID)

Ini adalah penyesuaian yang paling umum.

    File: src/robot_control/src/pid_x.py (Kemudi) & pid_y.py (Kecepatan Maju)

    Parameter: self.kp, self.kd

        kp (Proportional Gain): Anggap ini sebagai "pedal gas". Nilai yang lebih tinggi membuat robot merespons lebih cepat dan lebih agresif. Jika terlalu tinggi, robot akan menjadi tidak stabil dan berosilasi (gemetar).

        kd (Derivative Gain): Anggap ini sebagai "rem" atau "peredam kejut". Nilai yang lebih tinggi akan meredam osilasi dan mencegah robot overshoot (kebablasan). Jika terlalu tinggi, respons robot akan terasa lambat atau "berat".

2. Menyesuaikan Jarak Berhenti

    File: src/robot_control/src/object_detection.py

    Parameter: target_area_setpoint

    Nilai ini (dalam piksel persegi) menentukan seberapa besar plat hijau harus terlihat sebelum robot berhenti.

        Nilai Lebih Kecil (misal: 10000.0): Robot akan berhenti lebih jauh dari plat.

        Nilai Lebih Besar (misal: 40000.0): Robot akan berhenti lebih dekat dengan plat.

        Peringatan: Jika nilai ini terlalu besar, plat akan mulai terpotong oleh tepi kamera sebelum setpoint tercapai, yang menyebabkan perilaku tidak menentu (akselerasi mendadak).

3. Menyesuaikan Kecepatan Pencarian

    File: src/robot_control/src/movement_control.py

    Parameter: search_angular_velocity

    Nilai ini (dalam radian/detik) mengontrol seberapa cepat robot berputar di tempat saat mencari plat yang hilang.

Pengembangan di Masa Depan

    Integrasi Navigasi: Mengintegrasikan sensor Lidar dan paket robot_navigation untuk melakukan penghindaran rintangan (obstacle avoidance) saat mendekati target.

    SLAM: Menggunakan Lidar dan IMU untuk membangun peta lingkungan (SLAM) dan memungkinkan navigasi ke titik tujuan yang ditentukan.

    Kontroler PID Lanjutan: Mengimplementasikan anti-windup pada istilah Integral (Ki) untuk penyesuaian yang lebih halus dan menghilangkan steady-state error.

Lisensi

Proyek ini dilisensikan di bawah Lisensi MIT.

Lampiran:
1. Video Simulasi: https://drive.google.com/drive/folders/1HafsxIWreWxEfUF7UJ6uI1xf2absKVZc?usp=sharing
2. Alur Kerja dan Penjelasan: https://drive.google.com/drive/folders/194OHLaNlf3-DzptS3D1fzv_5t16KUpbB?usp=sharing
3. Repositori GitHub: https://github.com/Camn0/URO_ROS2_Gazebo/
