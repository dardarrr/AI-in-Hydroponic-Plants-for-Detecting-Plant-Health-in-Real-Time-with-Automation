import cv2

# Mencoba beberapa indeks untuk mencari kamera yang tersedia
for i in range(5):  # Coba indeks dari 0 hingga 4
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Kamera terdeteksi pada indeks {i}")
        cap.release()  # Tutup kamera setelah selesai
    else:
        print(f"Tidak ada kamera pada indeks {i}")
