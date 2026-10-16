import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# 1. Cargar los datos desde el archivo CSV generado por C++
data = pd.read_csv('simulation_results.csv')

# 2. Extraer las columnas
times = data['time']
Vout = data['Vout']
Vout_norm = data['Vout_normalized']

# -------------------------------------------------------------------------
# Figura 1: Evolución temporal del voltaje de salida normalizado (Vout / Vdd)
# -------------------------------------------------------------------------
plt.figure(figsize=(8, 4))
plt.plot(times, Vout_norm, color='tab:blue', linewidth=0.8)
plt.xlabel('time')
plt.ylabel(r'$V_\mathrm{out}/V_\mathrm{dd}$')
plt.title('Trayectoria estocástica del p-bit')
plt.grid(True, linestyle='--', alpha=0.5)
plt.tight_layout()
plt.show()

# -------------------------------------------------------------------------
# Figura 2: Histograma del voltaje de salida Vout
# -------------------------------------------------------------------------
plt.figure(figsize=(8, 4))
plt.hist(Vout, bins=70, color='tab:blue', edgecolor='black', alpha=0.7)
plt.xlabel(r'$V_\mathrm{out}$')
plt.ylabel('N')
plt.title(r'Distribución de voltaje $V_\mathrm{out}$')
plt.grid(True, linestyle='--', alpha=0.5)
plt.tight_layout()
plt.show()

