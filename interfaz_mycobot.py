#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState  # Necesario para el modo físico real
from builtin_interfaces.msg import Duration
import tkinter as tk
import numpy as np
import math
import threading

# =====================================================================
# CONFIGURACIÓN DE DESPLIEGUE (Cambia aquí según dónde trabajes)
# =====================================================================
MODO_TRABAJO = "SIMULACION"  # Opciones: "SIMULACION" (Gazebo) o "FISICO" (Jetson Nano Real)

# ==========================================
# 1. MOTOR MATEMÁTICO (Tu cinemática original)
# ==========================================
class CinematicaRobot:
    def __init__(self):
        self.H_SUELO_A_BASE = 9.0
        self.H_BASE_A_HOMBRO = 3.0
        self.H_PIVOTE = self.H_SUELO_A_BASE + self.H_BASE_A_HOMBRO
        self.L2, self.L3, self.L4_fijo = 8.3, 8.3, 7.3

    def forward_kinematics(self, angulos_rad):
        angulos_grados = [math.degrees(a) for a in angulos_rad]
        
        theta1 = np.radians(angulos_grados[0])
        q2 = np.radians(angulos_grados[1])
        q3 = np.radians(angulos_grados[2]) 
        q4 = np.radians(angulos_grados[3])

        r2, z2 = self.L2 * np.sin(q2), self.L2 * np.cos(q2)
        r3, z3 = self.L3 * np.sin(q2+q3), self.L3 * np.cos(q2+q3)
        r4, z4 = self.L4_fijo * np.sin(q2+q3+q4), self.L4_fijo * np.cos(q2+q3+q4)

        R_total = r2 + r3 + r4
        Z_total = self.H_PIVOTE + z2 + z3 + z4

        X_final = R_total * np.cos(theta1)
        Y_final = R_total * np.sin(theta1)
        
        return [X_final, Y_final, Z_total]

# ==========================================
# 2. NODO ROS 2 (Dual: Simulación / Físico)
# ==========================================
class RosControlNode(Node):
    def __init__(self, modo):
        super().__init__(node_name='advanced_gui_control')
        self.modo = modo
        
        # Mapeo de articulaciones según tu URDF verificado
        self.joint_names = [
            'link1_to_link2', 
            'link2_to_link3', 
            'link3_to_link4', 
            'link4_to_link5', 
            'link5_to_link6', 
            'link6_to_link6_flange'
        ]

        if self.modo == "SIMULACION":
            self.publisher_ = self.create_publisher(
                JointTrajectory, 
                '/arm_controller/joint_trajectory', 
                10
            )
            self.get_logger().info('Modo SIMULACIÓN activo: Publicando en /arm_controller')
        else:
            self.publisher_ = self.create_publisher(
                JointState, 
                '/joint_states', 
                10
            )
            self.get_logger().info('Modo FÍSICO real activo: Publicando en /joint_states')

    def publicar_trayectoria(self, angulos_rad, tiempo_segundos):
        """Envía comandos interpolados en el tiempo para el simulador físico."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        punto = JointTrajectoryPoint()
        punto.positions = list(angulos_rad)
        
        sec = int(tiempo_segundos)
        nanosec = int((tiempo_segundos - sec) * 1e9)
        punto.time_from_start = Duration(sec=sec, nanosec=nanosec)
        
        msg.points.append(punto)
        self.publisher_.publish(msg)

    def publicar_joint_state(self, angulos_rad):
        """Envía estados angulares directos para el hardware real del robot."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = list(angulos_rad)
        self.publisher_.publish(msg)

# ==========================================
# 3. INTERFAZ GRÁFICA COMPLETA
# ==========================================
class AppRobot(tk.Tk):
    def __init__(self, ros_node):
        super().__init__()
        self.title(f"MyCobot — Panel de Control ({ros_node.modo})")
        self.geometry("900x500")
        
        self.ros_node = ros_node
        self.kinematics = CinematicaRobot()
        self.angulos_rad = [0.0] * 6
        self.sliders = []
        self.bloqueo_slider = False
        
        self.crear_interfaz()

    def crear_interfaz(self):
        panel = tk.PanedWindow(self, orient=tk.HORIZONTAL)
        panel.pack(fill="both", expand=True, padx=10, pady=10)
        
        frame_L = tk.Frame(panel)
        panel.add(frame_L, stretch="always")
        tk.Label(frame_L, text="CONTROL MANUAL (Radianes)", bg="#333", fg="white").pack(fill="x")
        
        names = ["S1 (Base)", "S2 (Hombro)", "S3 (Codo)", "S4 (Muñeca)", "S5 (Giro)", "S6 (Flange)"]
        for i in range(6):
            f = tk.Frame(frame_L)
            f.pack(fill="x", pady=5)
            tk.Label(f, text=names[i], width=12, anchor="w").pack(side="left")
            s = tk.Scale(f, from_=-3.14, to=3.14, resolution=0.01, orient=tk.HORIZONTAL, length=200,
                         command=lambda v, idx=i: self.on_slider_change(idx, v))
            s.set(0.0)
            s.pack(side="left")
            self.sliders.append(s)

        frame_R = tk.Frame(panel)
        panel.add(frame_R, stretch="always")
        tk.Label(frame_R, text="AUTOMATIZACIÓN", bg="#333", fg="white").pack(fill="x")
        
        self.crear_boton(frame_R, "POSICIÓN HOME (0,0,0)", [0.0]*6, "#ccf")
        self.crear_boton(frame_R, "RECOGER PIEZA BAJA", [0.0, 0.5, -1.0, -0.5, 0.0, 0.0], "#fcc")
        self.crear_boton(frame_R, "MOSTRAR CÁMARA (Mirar Arriba)", [0.0, -0.5, 0.0, -1.5, 0.0, 0.0], "#cfc")

        tk.Label(frame_R, text="\nCoordenada del Efector (Cinemática Directa):").pack()
        self.lbl_xyz = tk.Label(frame_R, text="...", font=("Consolas", 14), relief="sunken", bg="#fff")
        self.lbl_xyz.pack(fill="x", pady=5)

    def crear_boton(self, parent, texto, angulos, color):
        tk.Button(parent, text=texto, bg=color, width=25, height=2, font=("Arial", 10, "bold"),
                  command=lambda: self.aplicar_movimiento_absoluto(angulos)).pack(pady=5)

    def on_slider_change(self, idx, val):
        if self.bloqueo_slider:
            return
        self.angulos_rad[idx] = float(val)
        self.actualizar_estado(tiempo_segundos=0.10)

    def aplicar_movimiento_absoluto(self, angulos_nuevos):
        self.bloqueo_slider = True
        for i in range(6):
            self.sliders[i].set(angulos_nuevos[i])
        self.bloqueo_slider = False
        
        self.angulos_rad = list(angulos_nuevos)
        self.actualizar_estado(tiempo_segundos=1.5)

    def actualizar_estado(self, tiempo_segundos):
        xyz = self.kinematics.forward_kinematics(self.angulos_rad)
        self.lbl_xyz.config(text=f"X:{xyz[0]:.2f} Y:{xyz[1]:.2f} Z:{xyz[2]:.2f}")
        
        # Publicar dinámicamente según el entorno configurado
        if self.ros_node.modo == "SIMULACION":
            self.ros_node.publicar_trayectoria(self.angulos_rad, tiempo_segundos)
        else:
            self.ros_node.publicar_joint_state(self.angulos_rad)

# ==========================================
# 4. LANZADOR MULTIHILO
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    ros_node = RosControlNode(modo=MODO_TRABAJO)
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    try:
        app = AppRobot(ros_node)
        app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        app.destroy()
        ros_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == "__main__":
    main()
