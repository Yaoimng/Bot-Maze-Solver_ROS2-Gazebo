# 🤖 Bot-Maze-Solver_ROS2-Gazebo

## 🎥 Visualisasi Simulasi

<!-- Ganti link gambar di bawah ini dengan GIF atau Screenshot demo Anda -->
![Demo Maze Solver](https://dummyimage.com/800x400/2b2b2b/ffffff&text=Masukan+GIF/Screenshot+Simulasi+Disini)

> *Robot bergerak secara otonom menelusuri labirin menggunakan sensor LIDAR di lingkungan Gazebo.*
---

## 🧠 Penjelasan Kode Penting

Sistem ini berjalan di atas **ROS 2** dengan dua komponen kode utama:

### **1. `simulation.launch.py` (Orkestrator)**
File ini menjalankan seluruh sistem secara bersamaan:
*   **Gazebo Server:** Memuat environment `maze.world`.
*   **Spawn Entity:** Memunculkan robot dari file URDF ke titik start.
*   **Wall Follower Node:** Mengaktifkan algoritma navigasi robot.

### **2. `wall_follower.py` (Otak Robot)**
Node ini bekerja menggunakan konsep *Publisher-Subscriber*:
*   **Subscribe `/scan`:** Menerima data jarak dari sensor LIDAR secara *real-time*.
*   **Publish `/cmd_vel`:** Mengirim perintah kecepatan (linear & angular) ke roda robot.

---

## 💡 Logika Utama (Wall Following Algorithm)

Robot membagi data sensor LIDAR menjadi tiga sektor: **Kiri**, **Depan**, dan **Kanan**. Keputusan gerak diambil berdasarkan prioritas keselamatan:

| Kondisi Sensor | Respon Robot |
| :--- | :--- |
| 🛑 **Depan Terhalang** | **Belok Kanan** (Menghindari tabrakan) |
| ⚠️ **Kiri Terlalu Dekat** | **Koreksi Kanan** (Menjauh dari dinding) |
| ⚠️ **Kanan Terlalu Dekat** | **Koreksi Kiri** (Menjaga posisi tengah) |
| ↩️ **Belokan Kiri Terdeteksi** | **Belok Kiri** (Mengikuti alur dinding) |
| ⬆️ **Jalan Aman** | **Maju Lurus** |

---

## 🏁 Kesimpulan

Proyek ini berhasil mendemonstrasikan navigasi robot otonom menggunakan pendekatan **Reaktif (Reactive Navigation)** tanpa peta global.

*   **Hasil:** Robot mampu menelusuri labirin dan menghindari tabrakan secara mandiri.
*   **Keterbatasan:** Karena hanya mengandalkan sensor lokal, robot mungkin kesulitan di jalan buntu yang kompleks.

### 👥 Kontributor
*Fadhil Zaky Budianto 5054231001*
*Dhafin Kurniawan 5054231016*
*Jeremia Kevin Alexander Jagardo Malau 5054231027*
*Alif As'ad Ramadhan 5054231007*
*Muhammad Naufal Arifin 5054231006*
*Bayu Nismara Nagatama 5025231152*
*Ignatius Devon Andri Putra 5025231159*
