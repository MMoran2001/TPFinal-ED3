import serial
import threading
import time

# -------------------------
# CONFIGURACIÓN
# -------------------------
COM_PORT = "COM9"
BAUD = 9600
TXT_INPUT = r"C:\Users\Marcos\OneDrive\UNC\Electronica digital III\RepoTpFinal\TPFinal-ED3\salida_2000.txt"
TXT_OUTPUT = r"C:\Users\Marcos\OneDrive\UNC\Electronica digital III\RepoTpFinal\TPFinal-ED3\recibido.txt"

# Abrir UART
ser = serial.Serial(
    port=COM_PORT,
    baudrate=BAUD,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.1
)

recibidos = []
ejecutando = True

# -------------------------
# HILO DE RECEPCIÓN
# -------------------------
def recibir():
    global ejecutando
    buffer = ""
    print("RX listo, esperando datos...\n")

    while ejecutando:
        if ser.in_waiting > 0:
            char = ser.read().decode(errors= 'ignore')
            if char in ["\n", "\r"]:
                if len(buffer)>0:
                    print(f"[RX] {buffer}")
                    recibidos.append(buffer)
                    buffer = ""
            else:
                buffer += char

    print("Recepción finalizada.")


# -------------------------
# FUNCIÓN DE ENVÍO
# -------------------------
def enviar():
    print("\nTX: Enviando datos...\n")

    # Cargar datos desde el TXT
    valores = []
    with open(TXT_INPUT, "r") as f:
        for line in f:
            v = int(line.strip())
            valores.append(max(0, min(255, v)))  # limitar a 0–255

    # Enviar uno por uno
    for v in valores:
        ser.write(bytes([v]))   # envía 1 byte
        print(f"[TX] {v}")
        time.sleep(0.001)       # delay opcional 1 ms

    print("\nTX completo.\n")


# -------------------------
# PROGRAMA PRINCIPAL
# -------------------------
try:
    # Lanzar hilo receptor
    hilo_rx = threading.Thread(target=recibir, daemon=True)
    hilo_rx.start()

    # Enviar datos
    enviar()

    print("Presioná CTRL+C para detener la recepción...")

    while True:
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nDeteniendo...")

finally:
    ejecutando = False
    time.sleep(0.2)
    ser.close()

    # Guarda
