#!/bin/bash
# =====================================================================
# Script para automatizar el arranque de la simulacion y la interfaz
# Creado por Jholaus Misael Villavicencio Lara
# =====================================================================

echo "====================================================================="
echo "   Iniciando Entorno de Simulacion ROS 2 - myCobot 280"
echo "====================================================================="

# 1. Abrir terminal para lanzar Gazebo (con renderizado por software para estabilidad)
echo "[INFO] Lanzando simulador fisico Gazebo..."
gnome-terminal --title="Gazebo - Motor Fisico" -- bash -c "
source /opt/ros/humble/setup.bash;
source ~/colcon_ws/install/setup.bash;
export LIBGL_ALWAYS_SOFTWARE=1;
ros2 launch mycobot_gazebo mycobot.gazebo.launch.py;
exec bash"

# 2. Esperar a que el entorno de fisica y los controladores esten activos
echo "[INFO] Esperando 8 segundos a que los controladores carguen..."
sleep 8

# 3. Abrir terminal para la interfaz grafica de control
echo "[INFO] Lanzando el Panel de Control Ciberfisico de Python..."
gnome-terminal --title="Interfaz de Control - Python" -- bash -c "
source /opt/ros/humble/setup.bash;
source ~/colcon_ws/install/setup.bash;
python3 ~/Documents/interfaz_mycobot.py;
exec bash"

echo "[OK] Todo listo. ¡A trabajar en la simulacion!"
