import serial
import time

# ----------------------------
# CONFIGURACIÓN
# ----------------------------
TXT_FILE = r"C:\Users\Marcos\OneDrive\UNC\Electronica digital III\RepoTpFinal\TPFinal-ED3\salida_2000.txt"        # Archivo con los valores
COM_PORT = "COM9"              # Cambiar por tu puerto
BAUD_RATE = 9600             # Velocidad UART
DELAY = 0.001                  # Retardo entre envíos (1 ms)

# ----------------------------
# ABRIR PUERTO SERIE
# ----------------------------
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print(f"Puerto {COM_PORT} abierto correctamente.")
except Exception as e:
    print("Error al abrir el puerto:", e)
    exit()

# ----------------------------
# LEER TXT Y ENVIAR UN VALOR POR VEZ
# ----------------------------
with open(TXT_FILE, "r") as f:
    for line in f:
        line = line.strip()
        if not line:
            continue

        # Si la línea contiene 2 números "L R", los separamos
        parts = line.split()

        for value in parts:   # enviar cada uno por separado
            to_send = value.encode('ascii') + b'\n'
            ser.write(to_send)
            time.sleep(DELAY)

ser.close()
print("Transferencia completa.")
