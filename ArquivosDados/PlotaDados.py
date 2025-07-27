import pandas as pd
import matplotlib.pyplot as plt
import os

script_dir = os.path.dirname(os.path.abspath(__file__))  # pega o diretório do script
csv_path = os.path.join(script_dir, 'MPU6050_data1.csv')  # monta o caminho completo para o CSV

df = pd.read_csv(csv_path)

# Extrai os dados relevantes
tempo = df['numero_amostra']
accel_x = df['accel_x']
accel_y = df['accel_y']
accel_z = df['accel_z']
giro_x = df['giro_x']
giro_y = df['giro_y']
giro_z = df['giro_z']

# Cria a figura com subplots
plt.figure(figsize=(12, 10))

# Aceleração
plt.subplot(2, 1, 1)
plt.plot(tempo, accel_x, label='Accel X', color='r')
plt.plot(tempo, accel_y, label='Accel Y', color='g')
plt.plot(tempo, accel_z, label='Accel Z', color='b')
plt.title("Aceleração nos 3 Eixos")
plt.xlabel("Número da Amostra")
plt.ylabel("Aceleração (g)")
plt.grid(True)
plt.legend()

# Giroscópio
plt.subplot(2, 1, 2)
plt.plot(tempo, giro_x, label='Giro X', color='c')
plt.plot(tempo, giro_y, label='Giro Y', color='m')
plt.plot(tempo, giro_z, label='Giro Z', color='y')
plt.title("Velocidade Angular nos 3 Eixos")
plt.xlabel("Número da Amostra")
plt.ylabel("Velocidade Angular (°/s)")
plt.grid(True)
plt.legend()

# Mostra tudo
plt.tight_layout()
plt.show()