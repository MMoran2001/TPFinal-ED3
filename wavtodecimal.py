
# Leer WAV
import scipy.io.wavfile as wav
import numpy as np

input_wav = r"C:\Users\Marcos\OneDrive\UNC\Electronica digital III\RepoTpFinal\TPFinal-ED3\audio.wav"       # <-- Cambiá por tu archivo
output_txt = r"C:\Users\Marcos\OneDrive\UNC\Electronica digital III\RepoTpFinal\TPFinal-ED3\salida.txt"  # <-- Cambiá por donde querés guardar

# Leer WAV
sr, data = wav.read(input_wav)

print("Frecuencia de muestreo:", sr)
print("Tipo de datos:", data.dtype)

# Si el audio NO es estéreo, detener (porque el usuario pidió mantener estéreo)
if len(data.shape) < 2 or data.shape[1] != 2:
    raise ValueError("El archivo WAV no es estéreo (2 canales).")

# Extraer canales
left = data[:, 0]
right = data[:, 1]

# Escalar ambos canales a rango 0–4095
def scale_to_4095(x):
    x = x.astype(float)
    x_min = x.min()
    x_max = x.max()
    scaled = (x - x_min) / (x_max - x_min) * 255.0
    return scaled.astype(int)

left_4095 = scale_to_4095(left)
right_4095 = scale_to_4095(right)

# Guardar en TXT, formato: "L R"
with open(output_txt, "w") as f:
    for l, r in zip(left_4095, right_4095):
        f.write(f"{l} {r}\n")

print("Listo! Exportado a:", output_txt)

