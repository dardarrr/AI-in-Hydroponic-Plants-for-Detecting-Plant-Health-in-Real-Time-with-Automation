from flask import Flask, Response, render_template, jsonify
import cv2
from ultralytics import YOLO
import serial
import time
import atexit
import serial.tools.list_ports
from threading import Thread

app = Flask(__name__)

# Inisialisasi model YOLOv8 nano (versi lebih ringan)
model = YOLO("hidroponik.onnx")  # Ganti dengan model YOLOv8 yang lebih ringan

# Inisialisasi serial untuk komunikasi dengan Arduino
arduino = None

# Variabel global untuk status relay dan label objek
relay_status = "Off"
detected_label = "Tidak Terdeteksi"
relay_last_activated = 0  # Waktu ketika relay terakhir kali diaktifkan
relay_duration = 5000  # Durasi relay menyala dalam milidetik (5 detik)

# Fungsi untuk mendeteksi port Arduino secara otomatis
def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'Arduino' in port.description:
            return port.device
    return None

# Inisialisasi koneksi serial dengan Arduino
def init_serial():
    global arduino
    try:
        port = find_arduino_port()
        if port is not None:
            arduino = serial.Serial(port, 9600, timeout=5)  # Perbesar timeout jika perlu
            time.sleep(2)  # Tunggu agar koneksi serial stabil
            print(f"Koneksi serial berhasil pada port {port}!")
        else:
            print("Arduino tidak ditemukan. Pastikan sudah terhubung.")
    except serial.SerialException as se:
        print(f"SerialException: {se}")
    except Exception as e:
        print(f"Error membuka koneksi serial: {e}")

# Menutup koneksi serial saat aplikasi berhenti
def close_serial():
    global arduino
    if arduino is not None and arduino.is_open:
        arduino.close()
        print("Koneksi serial ditutup.")

# Daftarkan close_serial agar selalu dipanggil ketika aplikasi berhenti
atexit.register(close_serial)

# VideoStream class untuk menangani pengambilan frame menggunakan multithreading
class VideoStream:
    def __init__(self, source=1):
        self.camera = cv2.VideoCapture(source, cv2.CAP_DSHOW)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Kurangi resolusi untuk mempercepat
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
        self.ret, self.frame = self.camera.read()
        self.stopped = False

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                self.camera.release()
                return
            self.ret, self.frame = self.camera.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True

# Fungsi untuk membaca video dari webcam dan menjalankan deteksi objek
def gen_frames():
    global detected_label, relay_status, relay_last_activated
    video_stream = VideoStream(1).start()  # Memulai stream video
    while True:
        frame = video_stream.read()  # Baca frame dari stream
        if frame is None:
            print("Error: Tidak bisa membaca frame dari webcam.")
            break

        results = model(frame, conf=0.7)  # Deteksi objek dengan YOLOv8
        annotated_frame = frame.copy()  # Salin frame untuk anotasi

        detected_label = "Tidak Terdeteksi"  # Reset label sebelum deteksi baru

        # Iterasi melalui hasil deteksi
        for result in results:
            boxes = result.boxes  # Mendapatkan kotak pembatas dari deteksi

            # Iterasi setiap kotak pembatas
            for box in boxes:
                # Mendapatkan koordinat kotak, label, dan confidence score
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Koordinat bounding box
                label = result.names[int(box.cls[0])]  # Nama kelas
                confidence = box.conf[0]  # Confidence score

                # Cetak informasi deteksi untuk debugging
                print(f"Label yang terdeteksi: {label}, Confidence: {confidence:.2f}")

                # Gambar bounding box di frame
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Kotak hijau
                cv2.putText(annotated_frame, f'{label} {confidence:.2f}', (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                # Jika kelas "Kurang Nutrisi" terdeteksi, kirim sinyal ke Arduino
                if label == "Kurang Nutrisi" and relay_status == "Off":
                    print("Tanaman kurang nutrisi terdeteksi, mengirim sinyal ke Arduino.")
                    send_signal_to_arduino()
                    relay_last_activated = time.time() * 1000  # Simpan waktu relay aktif
                    relay_status = "On"  # Update status relay

                detected_label = label  # Update label terdeteksi

        # Mematikan relay setelah relay_duration
        if relay_status == "On" and (time.time() * 1000 - relay_last_activated) > relay_duration:
            print("Mematikan relay setelah durasi.")
            send_signal_to_arduino_off()
            relay_status = "Off"  # Reset status relay agar bisa diaktifkan lagi

        # Encode frame ke format JPEG sebelum dikirim ke web
        ret, buffer = cv2.imencode('.jpg', annotated_frame)
        if not ret:
            print("Error: Tidak bisa meng-encode frame.")
            continue
        
        frame = buffer.tobytes()

        # Mengirim frame ke browser
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# Fungsi untuk mengirim sinyal "ON" ke Arduino via port serial
def send_signal_to_arduino():
    global arduino
    if arduino is not None and arduino.is_open:
        try:
            arduino.write(b'ON\n')  # Mengirimkan sinyal untuk menyalakan pompa
            print("Sinyal 'ON' dikirim ke Arduino.")
        except serial.SerialTimeoutException as ste:
            print(f"SerialTimeoutException: {ste}")
        except Exception as e:
            print(f"Error mengirim sinyal ke Arduino: {e}")
    else:
        print("Koneksi serial belum tersedia atau belum terbuka.")

# Fungsi untuk mengirim sinyal "OFF" ke Arduino via port serial
def send_signal_to_arduino_off():
    global arduino
    if arduino is not None and arduino.is_open:
        try:
            arduino.write(b'OFF\n')  # Mengirimkan sinyal untuk mematikan pompa
            print("Sinyal 'OFF' dikirim ke Arduino.")
        except serial.SerialTimeoutException as ste:
            print(f"SerialTimeoutException: {ste}")
        except Exception as e:
            print(f"Error mengirim sinyal ke Arduino: {e}")
    else:
        print("Koneksi serial belum tersedia atau belum terbuka.")

# Rute untuk menampilkan video feed
@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Rute untuk menampilkan halaman HTML
@app.route('/')
def index():
    return render_template('index.html')  # Mengarahkan ke file index.html

# Rute untuk mendapatkan data status dan label
@app.route('/status')
def status():
    return jsonify({
        'label': detected_label,
        'relay_status': relay_status
    })

# Entry point aplikasi
if __name__ == '__main__':
    init_serial()
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)  # Matikan auto-reload
    finally:
        close_serial()
