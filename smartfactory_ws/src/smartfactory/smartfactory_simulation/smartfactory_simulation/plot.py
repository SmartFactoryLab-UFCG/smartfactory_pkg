import csv
import matplotlib.pyplot as plt
import numpy as np

# Caminho para o arquivo CSV
csv_file_path = '/workspaces/smartfactory_pkg/smartfactory_ws/aruco_pose_data.csv'

# Listas para armazenar os dados do CSV em centímetros
x_data = []
y_data = []
z_data = []

# Ler o arquivo CSV
with open(csv_file_path, mode='r') as csv_file:
    csv_reader = csv.reader(csv_file)
    next(csv_reader)  # Pular a primeira linha (cabeçalhos)
    
    for row in csv_reader:
        # Convertendo os dados de metros para centímetros
        x_data.append(float(row[0]) * 100)  # Coluna X em cm
        y_data.append(float(row[1]) * 100)  # Coluna Y em cm
        z_data.append(float(row[2]) * 100)  # Coluna Z em cm

# Função para calcular e exibir métricas
def calcular_metricas(dados, eixo):
    media = np.mean(dados)
    variancia = np.var(dados)
    desvio_padrao = np.std(dados)
    rmse = np.sqrt(np.mean((dados - media) ** 2))
    mae = np.mean(np.abs(dados - media))

    print(f'\nMétricas para o eixo {eixo}:')
    print(f'Média: {media:.4f} cm')
    print(f'Variância: {variancia:.4f} cm²')
    print(f'Desvio Padrão: {desvio_padrao:.4f} cm')
    print(f'Erro Quadrático Médio (RMSE): {rmse:.4f} cm')
    print(f'Erro Absoluto Médio (MAE): {mae:.4f} cm')

# Calcular e exibir as métricas para X, Y e Z
calcular_metricas(x_data, 'X')
calcular_metricas(y_data, 'Y')
calcular_metricas(z_data, 'Z')

# Criar os subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))

# Plotar X (em cm)
ax1.plot(x_data, label='Posição X', color='r')
ax1.set_title('Posição X ao longo do tempo')
ax1.set_ylabel('Posição X (cm)')
ax1.set_ylim([min(x_data) - 0.1, max(x_data) + 0.1])  # Ajuste fino para X
ax1.grid(True)

# Plotar Y (em cm)
ax2.plot(y_data, label='Posição Y', color='g')
ax2.set_title('Posição Y ao longo do tempo')
ax2.set_ylabel('Posição Y (cm)')
ax2.set_ylim([min(y_data) - 0.1, max(y_data) + 0.1])  # Ajuste fino para Y
ax2.grid(True)

# Plotar Z (em cm)
ax3.plot(z_data, label='Posição Z', color='b')
ax3.set_title('Posição Z ao longo do tempo')
ax3.set_ylabel('Posição Z (cm)')
ax3.set_xlabel('Amostras (tempo)')
ax3.set_ylim([min(z_data) - 0.1, max(z_data) + 0.1])  # Ajuste fino para Z
ax3.grid(True)

# Ajustar o layout
plt.tight_layout()

# Mostrar o gráfico
plt.show()