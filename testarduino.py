import serial

try:
    arduino = serial.Serial('COM8', 9600)  # Ganti COM dengan port yang sesuai
    print("Koneksi berhasil!")
    arduino.close()
except Exception as e:
    print(f"Error: {e}")
