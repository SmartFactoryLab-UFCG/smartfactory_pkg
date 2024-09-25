import cv2
import numpy as np
import glob

# Função para calibrar a câmera e retornar a matriz intrínseca
def calibrate_camera(image_folder, checkerboard_size=(8, 6), square_size=0.058):
    # Definir critérios de parada para o refinamento de cantos
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Preparar os pontos de objeto, como (0,0,0), (1,0,0), (2,0,0), ..., (7,5,0)
    objp = np.zeros((np.prod(checkerboard_size), 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # Armazenar pontos de objeto e pontos de imagem de todas as imagens
    objpoints = []  # Pontos 3D no mundo real
    imgpoints = []  # Pontos 2D no plano da imagem

    # Caminho para as imagens
    image_pattern = image_folder + 'left-*.png'

    # Carregar e processar as imagens capturadas
    images = sorted(glob.glob(image_pattern))

    gray = None  # Definir uma variável gray inicial para garantir que esteja disponível após o loop

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Encontrar os cantos do tabuleiro de xadrez
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

        if ret:
            objpoints.append(objp)  # Adiciona pontos 3D conhecidos (padrão do mundo)
            
            # Refina os cantos para maior precisão
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Mostrar as imagens com os cantos detectados (opcional)
            cv2.drawChessboardCorners(img, checkerboard_size, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()

    # Verificar se pelo menos uma imagem foi processada antes de calibrar
    if len(objpoints) > 0 and gray is not None:
        # Calibração da câmera para obter os parâmetros intrínsecos e extrínsecos
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        # Retorna a matriz de calibração (intrínseca) e os coeficientes de distorção
        return mtx, dist
    else:
        print("Nenhuma imagem de tabuleiro de xadrez válida foi encontrada.")
        return None, None


# Usar a função para calibrar a câmera e obter a matriz intrínseca
image_folder = '/workspaces/smartfactory_pkg/smartfactory_ws/src/calibrationdata/'
intrinsic_matrix, distortion_coeffs = calibrate_camera(image_folder)

if intrinsic_matrix is not None:
    print("Matriz de Calibração (Intrínseca):\n", intrinsic_matrix)
    print("Coeficientes de Distorção:\n", distortion_coeffs)
else:
    print("Erro na calibração.")
