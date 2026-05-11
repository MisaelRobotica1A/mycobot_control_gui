#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk
from tkinter import ttk
import numpy as np
import math
import threading

# ==========================================
# 1. MOTOR MATEMÁTICO
# ==========================================
class CinematicaRobot:
    def __init__(self):
        self.H_SUELO_A_BASE = 9.0
        self.H_BASE_A_HOMBRO = 3.0
        self.H_PIVOTE = self.H_SUELO_A_BASE + self.H_BASE_A_HOMBRO
        self.L2, self.L3, self.L4_fijo = 8.3, 8.3, 7.3

    def forward_kinematics(self, angulos_rad):
        theta1 = angulos_rad[0]
        q2     = angulos_rad[1]
        q3     = angulos_rad[2]
        q4     = angulos_rad[3]

        r2 = self.L2 * np.sin(q2);     z2 = self.L2 * np.cos(q2)
        r3 = self.L3 * np.sin(q2+q3);  z3 = self.L3 * np.cos(q2+q3)
        r4 = self.L4_fijo * np.sin(q2+q3+q4)
        z4 = self.L4_fijo * np.cos(q2+q3+q4)

        R_total = r2 + r3 + r4
        Z_total = self.H_PIVOTE + z2 + z3 + z4
        X_final = R_total * np.cos(theta1)
        Y_final = R_total * np.sin(theta1)
        return [X_final, Y_final, Z_total]


# ==========================================
# 2. NODO ROS 2
# ==========================================
class RosControlNode(Node):
    def __init__(self):
        super().__init__(node_name='advanced_gui_control')
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        self.joint_names = [
            'link1_to_link2', 'link2_to_link3', 'link3_to_link4',
            'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange'
        ]

    def publicar_trayectoria(self, angulos_rad, tiempo_segundos):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        punto = JointTrajectoryPoint()
        punto.positions = list(angulos_rad)
        sec    = int(tiempo_segundos)
        nanosec = int((tiempo_segundos - sec) * 1e9)
        punto.time_from_start = Duration(sec=sec, nanosec=nanosec)
        msg.points.append(punto)
        self.publisher_.publish(msg)


# ==========================================
# 3. INTERFAZ GRÁFICA — BOTONES DIRECCIONALES
# ==========================================
DARK_BG   = "#1a1a1a"
PANEL_BG  = "#242424"
BTN_BG    = "#2e2e2e"
BTN_ACT   = "#0a7ea4"   # azul al presionar
TEXT_PRI  = "#e8e8e8"
TEXT_SEC  = "#888888"
ACCENT_R  = "#e24b4a"
ACCENT_G  = "#1d9e75"
ACCENT_B  = "#378add"
FONT_MONO = ("Courier New", 10)
FONT_BOLD = ("Courier New", 10, "bold")
FONT_LG   = ("Courier New", 14, "bold")


class BotonDir(tk.Button):
    """Botón direccional con feedback visual de pulsación."""

    def __init__(self, parent, symbol, command, **kw):
        super().__init__(
            parent, text=symbol,
            font=("Courier New", 16), width=3, height=1,
            bg=BTN_BG, fg=TEXT_PRI, relief="flat",
            activebackground=BTN_ACT, activeforeground="white",
            cursor="hand2",
            command=command,
            **kw
        )
        self.bind("<ButtonPress-1>",   lambda e: self._press())
        self.bind("<ButtonRelease-1>", lambda e: self._release())

    def _press(self):
        self.config(bg=BTN_ACT, fg="white")

    def _release(self):
        self.config(bg=BTN_BG, fg=TEXT_PRI)


class AppRobot(tk.Tk):

    PASOS = {"fino": 0.05, "medio": 0.15, "rápido": 0.35}
    PRESETS = {
        "🏠  Home":          [0.0, 0.0,  0.0,  0.0,  0.0, 0.0],
        "📦  Recoger pieza": [0.0, 0.5, -1.0, -0.5,  0.0, 0.0],
        "📷  Mostrar cámara":[0.0,-0.5,  0.0, -1.5,  0.0, 0.0],
        "↔️  Posición lateral":[1.57,-0.3,-0.5, 0.0, 0.0, 0.0],
    }

    def __init__(self, ros_node):
        super().__init__()
        self.title("MyCobot — Panel de Control")
        self.configure(bg=DARK_BG)
        self.resizable(False, False)

        self.ros_node   = ros_node
        self.kinematics = CinematicaRobot()
        self.angulos    = [0.0] * 6
        self.paso       = 0.15
        self._hold_job  = None

        self._construir_ui()
        self._actualizar_display()

    # --------------------------------------------------
    # CONSTRUCCIÓN DE LA UI
    # --------------------------------------------------
    def _construir_ui(self):
        # Título
        tk.Label(self, text="MYCOBOT  CONTROL  PANEL",
                 bg=DARK_BG, fg=TEXT_SEC, font=("Courier New", 9),
                 pady=8).pack(fill="x")

        contenido = tk.Frame(self, bg=DARK_BG)
        contenido.pack(padx=12, pady=(0, 12))

        # Columna izquierda: D-pads
        col_l = tk.Frame(contenido, bg=DARK_BG)
        col_l.pack(side="left", anchor="n")

        self._crear_dpad_xy(col_l)
        tk.Frame(col_l, bg=DARK_BG, height=10).pack()
        self._crear_dpad_z(col_l)
        tk.Frame(col_l, bg=DARK_BG, height=10).pack()
        self._crear_velocidad(col_l)

        # Separador
        tk.Frame(contenido, bg=PANEL_BG, width=1).pack(side="left", fill="y", padx=12)

        # Columna derecha: telemetría + presets
        col_r = tk.Frame(contenido, bg=DARK_BG)
        col_r.pack(side="left", anchor="n")

        self._crear_telemetria(col_r)
        tk.Frame(col_r, bg=DARK_BG, height=10).pack()
        self._crear_presets(col_r)

    # -- D-PAD XY --
    def _crear_dpad_xy(self, parent):
        f = tk.Frame(parent, bg=PANEL_BG, padx=10, pady=10, relief="flat")
        f.pack()
        tk.Label(f, text="TRASLACIÓN  XY", bg=PANEL_BG, fg=TEXT_SEC,
                 font=("Courier New", 8), pady=4).grid(row=0, column=0, columnspan=3)

        BotonDir(f, "▲", lambda: self._mover("adelante")).grid(row=1, column=1, padx=2, pady=2)
        BotonDir(f, "◀", lambda: self._mover("izquierda")).grid(row=2, column=0, padx=2, pady=2)

        # Botón HOME central
        tk.Button(f, text="H", font=FONT_BOLD, width=3, height=1,
                  bg=ACCENT_R, fg="white", relief="flat",
                  command=self._home, cursor="hand2"
                  ).grid(row=2, column=1, padx=2, pady=2)

        BotonDir(f, "▶", lambda: self._mover("derecha")).grid(row=2, column=2, padx=2, pady=2)
        BotonDir(f, "▼", lambda: self._mover("atras")).grid(row=3, column=1, padx=2, pady=2)

        tk.Label(f, text="adelante · atrás · izq · der",
                 bg=PANEL_BG, fg=TEXT_SEC, font=("Courier New", 7)).grid(row=4, column=0, columnspan=3)

    # -- D-PAD Z --
    def _crear_dpad_z(self, parent):
        f = tk.Frame(parent, bg=PANEL_BG, padx=10, pady=10)
        f.pack()
        tk.Label(f, text="ALTURA  Z", bg=PANEL_BG, fg=TEXT_SEC,
                 font=("Courier New", 8), pady=4).pack()
        BotonDir(f, "▲", lambda: self._mover("subir")).pack(pady=2)
        tk.Label(f, text="─  Z  ─", bg=PANEL_BG, fg=TEXT_SEC,
                 font=("Courier New", 9)).pack()
        BotonDir(f, "▼", lambda: self._mover("bajar")).pack(pady=2)

    # -- VELOCIDAD --
    def _crear_velocidad(self, parent):
        f = tk.Frame(parent, bg=PANEL_BG, padx=10, pady=8)
        f.pack(fill="x")
        tk.Label(f, text="PASO  DE  MOVIMIENTO", bg=PANEL_BG, fg=TEXT_SEC,
                 font=("Courier New", 8), pady=4).pack()
        self._btn_paso = {}
        row = tk.Frame(f, bg=PANEL_BG)
        row.pack()
        for nombre, valor in self.PASOS.items():
            b = tk.Button(row, text=nombre.upper(), font=("Courier New", 8),
                          bg=BTN_BG, fg=TEXT_SEC, relief="flat",
                          cursor="hand2", padx=6, pady=4,
                          command=lambda v=valor, n=nombre: self._set_paso(v, n))
            b.pack(side="left", padx=2)
            self._btn_paso[nombre] = b
        self._set_paso(0.15, "medio")

    # -- TELEMETRÍA --
    def _crear_telemetria(self, parent):
        f = tk.Frame(parent, bg=PANEL_BG, padx=12, pady=10)
        f.pack(fill="x")
        tk.Label(f, text="POSICIÓN  DEL  EFECTOR  (FK)", bg=PANEL_BG,
                 fg=TEXT_SEC, font=("Courier New", 8), pady=4).pack(anchor="w")

        self._lbl = {}
        for eje, color in [("X", ACCENT_R), ("Y", ACCENT_G), ("Z", ACCENT_B)]:
            row = tk.Frame(f, bg=PANEL_BG)
            row.pack(fill="x", pady=2)
            tk.Label(row, text=eje, bg=PANEL_BG, fg=color,
                     font=FONT_BOLD, width=2).pack(side="left")
            lbl = tk.Label(row, text="  0.00 cm", bg=PANEL_BG,
                           fg=TEXT_PRI, font=FONT_LG, anchor="e")
            lbl.pack(side="right")
            self._lbl[eje] = lbl

        # Ángulos articulares
        tk.Label(f, text="ÁNGULOS  ARTICULARES  (rad)", bg=PANEL_BG,
                 fg=TEXT_SEC, font=("Courier New", 8), pady=6).pack(anchor="w", pady=(10, 0))
        self._lbl_j = []
        grid = tk.Frame(f, bg=PANEL_BG)
        grid.pack()
        for i in range(6):
            tk.Label(grid, text=f"J{i+1}", bg=PANEL_BG, fg=TEXT_SEC,
                     font=FONT_MONO, width=3).grid(row=i//3, column=(i%3)*2, sticky="e")
            lbl = tk.Label(grid, text=" 0.00", bg=PANEL_BG, fg=TEXT_PRI,
                           font=FONT_MONO, width=6, anchor="w")
            lbl.grid(row=i//3, column=(i%3)*2+1, padx=(0, 8))
            self._lbl_j.append(lbl)

    # -- PRESETS --
    def _crear_presets(self, parent):
        f = tk.Frame(parent, bg=PANEL_BG, padx=12, pady=10)
        f.pack(fill="x")
        tk.Label(f, text="POSICIONES  PRESET", bg=PANEL_BG,
                 fg=TEXT_SEC, font=("Courier New", 8), pady=4).pack(anchor="w")
        for nombre, angulos in self.PRESETS.items():
            tk.Button(f, text=nombre, font=("Courier New", 10),
                      bg=BTN_BG, fg=TEXT_PRI, relief="flat",
                      cursor="hand2", anchor="w", padx=8, pady=6,
                      activebackground=BTN_ACT, activeforeground="white",
                      command=lambda a=angulos, n=nombre: self._aplicar_preset(a, n)
                      ).pack(fill="x", pady=2)

    # --------------------------------------------------
    # LIMITES REALES DEL MYCOBOT 280 (Elephant Robotics)
    # J1-J5: +-165 grados  |  J6: +-179 grados
    # Se aplica margen de seguridad de 5 grados adicional
    # --------------------------------------------------
    LIMITES = [
        (-math.radians(160), math.radians(160)),  # J1 base
        (-math.radians(160), math.radians(160)),  # J2 hombro
        (-math.radians(160), math.radians(160)),  # J3 codo
        (-math.radians(160), math.radians(160)),  # J4 muneca
        (-math.radians(160), math.radians(160)),  # J5 giro
        (-math.radians(174), math.radians(174)),  # J6 flange
    ]

    # Tiempo minimo por articulacion (segundos).
    # J1 necesita mas tiempo: tiene mayor torque con el brazo extendido.
    TIEMPO_MIN = [0.25, 0.15, 0.15, 0.12, 0.12, 0.12]

    # --------------------------------------------------
    # LOGICA DE MOVIMIENTO
    # --------------------------------------------------
    def _mover(self, direccion):
        """Modifica los angulos segun la direccion presionada."""
        p = self.paso
        a = self.angulos

        if   direccion == "adelante":
            a[1] = self._clamp_j(1, a[1] - p);  tiempo = self.TIEMPO_MIN[1]
        elif direccion == "atras":
            a[1] = self._clamp_j(1, a[1] + p);  tiempo = self.TIEMPO_MIN[1]
        elif direccion == "izquierda":
            a[0] = self._clamp_j(0, a[0] + p);  tiempo = self.TIEMPO_MIN[0]
        elif direccion == "derecha":
            a[0] = self._clamp_j(0, a[0] - p);  tiempo = self.TIEMPO_MIN[0]
        elif direccion == "subir":
            a[2] = self._clamp_j(2, a[2] - p);  tiempo = self.TIEMPO_MIN[2]
        elif direccion == "bajar":
            a[2] = self._clamp_j(2, a[2] + p);  tiempo = self.TIEMPO_MIN[2]
        else:
            tiempo = 0.15

        self._publicar_y_mostrar(tiempo=tiempo)

    def _home(self):
        self.angulos = [0.0] * 6
        self._publicar_y_mostrar(tiempo=1.5)

    def _aplicar_preset(self, angulos, nombre):
        # Verificar limites antes de enviar
        self.angulos = [self._clamp_j(i, v) for i, v in enumerate(angulos)]
        self._publicar_y_mostrar(tiempo=1.8)

    def _set_paso(self, valor, nombre):
        self.paso = valor
        for n, b in self._btn_paso.items():
            b.config(bg=BTN_ACT if n == nombre else BTN_BG,
                     fg="white"  if n == nombre else TEXT_SEC)

    def _clamp_j(self, idx, v):
        """Clamp con los limites reales de cada articulacion del MyCobot 280."""
        lo, hi = self.LIMITES[idx]
        return max(lo, min(hi, v))

    @staticmethod
    def _clamp(v, lo=-math.pi, hi=math.pi):
        return max(lo, min(hi, v))

    def _publicar_y_mostrar(self, tiempo):
        self._actualizar_display()
        self.ros_node.publicar_trayectoria(self.angulos, tiempo)

    def _actualizar_display(self):
        xyz = self.kinematics.forward_kinematics(self.angulos)
        ejes = ["X", "Y", "Z"]
        for i, eje in enumerate(ejes):
            self._lbl[eje].config(text=f"{xyz[i]:+7.2f} cm")
        for i, lbl in enumerate(self._lbl_j):
            lbl.config(text=f"{self.angulos[i]:+5.2f}")


# ==========================================
# 4. LANZADOR MULTIHILO
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    ros_node = RosControlNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    try:
        app = AppRobot(ros_node)
        app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2)


if __name__ == "__main__":
    main()
