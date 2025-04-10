import os
import subprocess

def run_ros2_command(package, executable):
    """
    Executa um comando `ros2 run` garantindo que ele herde corretamente o ROS_DOMAIN_ID.

    Args:
        package (str): Nome do pacote ROS2.
        executable (str): Nome do nó/executável a ser rodado.

    Returns:
        subprocess.Popen: Processo iniciado.
    """
    ros_domain_id = os.environ.get("ROS_DOMAIN_ID", "10")  # Obtém o ROS_DOMAIN_ID do ambiente (padrão 10)
    env = os.environ.copy()  # Copia o ambiente atual
    env["ROS_DOMAIN_ID"] = ros_domain_id  # Garante que o subprocesso roda no mesmo domínio

    process = subprocess.Popen(['ros2', 'run', package, executable], env=env)

    print(f"Executando '{package} {executable}' com ROS_DOMAIN_ID={ros_domain_id}")
    return process
