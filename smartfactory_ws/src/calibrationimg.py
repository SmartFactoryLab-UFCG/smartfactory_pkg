import cv2
import numpy as np
import glob
import os

# Dimensões do tabuleiro de xadrez (11x12 cruzamentos internos)
CHECKERBOARD = (11, 12)
SQUARE_SIZE = 0.03  # Tamanho de cada quadrado em metros

# Critérios de término para o algoritmo de detecção de cantos
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Preparar os pontos de objeto, por exemplo, (0,0,0), (1,0,0), ..., (10,11,0)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

# Vetores para armazenar pontos 3D e pontos 2D de todas as imagens
objpoints = []  # Pontos no espaço 3D
imgpoints = []  # Pontos no espaço 2D

# Caminho para a pasta com as imagens e a pasta para salvar imagens com pontos detectados
image_folder = '/workspaces/smartfactory_pkg/smartfactory_ws/src/etc/calibrationdata_new'
save_folder = '/workspaces/smartfactory_pkg/smartfactory_ws/src/etc/calibration_results'
os.makedirs(save_folder, exist_ok=True)

# Loop por todas as imagens na pasta
images = glob.glob(f'{image_folder}/*.png')
image_size = None  # Variável para armazenar o tamanho da imagem

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Encontrar os cantos do tabuleiro de xadrez
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    # Se os cantos foram encontrados, adicionar pontos de objeto e imagem
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Armazenar o tamanho da imagem (necessário para a calibração)
        if image_size is None:
            image_size = gray.shape[::-1]

        # Desenhar os cantos detectados e salvar a imagem
        img_with_corners = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        save_path = os.path.join(save_folder, os.path.basename(fname))
        cv2.imwrite(save_path, img_with_corners)

cv2.destroyAllWindows()

# Verificar se foram encontradas imagens válidas para calibração
if len(objpoints) > 0 and len(imgpoints) > 0:
    # Calibrar a câmera usando os pontos detectados
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)

    # Exibir os resultados da calibração
    print("Matriz da Câmera:\n", mtx)
    print("Coeficientes de Distorção:\n", dist)
    print("Vetores de rotação:\n", rvecs)
    print("Vetores de translação:\n", tvecs)

    # Salvar os resultados para uso futuro
    np.savez('/workspaces/smartfactory_pkg/smartfactory_ws/src/etc/calibration_parameters.npz', 
             camera_matrix=mtx, dist_coeff=dist, rvecs=rvecs, tvecs=tvecs)

    print("Calibração concluída, imagens salvas com pontos detectados e parâmetros de calibração armazenados!")
else:
    print("Nenhuma imagem válida encontrada para calibração. Verifique se as imagens contêm o tabuleiro de xadrez corretamente.")
