import numpy as np

def calculate_transformation_svd(P_camera, P_world):
    # Número de pontos
    assert P_camera.shape == P_world.shape, "Os dois conjuntos de pontos devem ter o mesmo tamanho"
    n = P_camera.shape[0]

    # Calcular a média de ambos os conjuntos de pontos
    centroid_camera = np.mean(P_camera, axis=0)
    centroid_world = np.mean(P_world, axis=0)

    # Centralizar os pontos subtraindo a média
    P_camera_centered = P_camera - centroid_camera
    P_world_centered = P_world - centroid_world

    # Calcular a matriz de covariância
    H = P_camera_centered.T @ P_world_centered

    # Aplicar SVD na matriz de covariância
    U, S, Vt = np.linalg.svd(H)

    # Calcular a matriz de rotação
    R = Vt.T @ U.T

    # Garantir que a matriz de rotação seja válida (evitar reflexão)
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T

    # Calcular o vetor de translação
    t = centroid_world.T - R @ centroid_camera.T

    # Criar a matriz de transformação homogênea 4x4
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T

# Exemplo de uso:

# Conjunto de pontos no sistema da câmera (em metros)
P_camera = np.array([
    [0.5, 0.0, 0.0],
    [0.0, 0.5, 0.0],
    [0.0, 0.0, 0.5]
])

# Conjunto de pontos correspondentes no sistema do mundo (base do robô) (em metros)
P_world = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0]
])

# Calcular a matriz de transformação
T = calculate_transformation_svd(P_camera, P_world)

# Exibir a matriz de transformação
print("Matriz de Transformação (R e t):")
print(T)
