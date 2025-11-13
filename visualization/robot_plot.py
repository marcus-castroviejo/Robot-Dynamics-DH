"""
=================================================================================================================
                                                Classe RobotPlot
=================================================================================================================
- Plots para o Painel de Visualização da Interface
    - 4 plots: [plot 3D, vista superior XY, vista lateral RZ, q(t) X t]

Campos deste código:
["Setup Inicial"]:          Inicialização, Criação dos plots
["Criação dos Plots"]:      Configuração dos plots e das linhas, Posicionamento na Grid
["Dados de Referência"]:    Plotar os dados de referencia: [workspace, target, trajetória desejada]
["Funções de Update"]:      Atualização dos plots em tempo real 
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Wedge
from matplotlib.gridspec import GridSpec
from PyQt6.QtWidgets import QWidget


class RobotPlot(QWidget):
    """Visualização para a Interface"""
    
    """
    =================================================================================================================
                                                Setup inicial
    =================================================================================================================
    """

    """--------------------------- __init__() ---------------------------"""
    def __init__(self, robot, parent=None):
        super().__init__()
        self.robot = robot
        self.initialize_data()              # Salvar dados temporais
        
        try:
            self.create_figures()           # Cria uma figura para cada Aba
            self.setup_plots()              # Configura: posição, title, labels, limits, grid, ...
            self.init_plot_lines()          # Inicializa as linhas dos plots
            self.plot_workspace_views()     # Desenha o Workspace            
        except Exception as e:
            print(f"Erro ao inicializar plot: {e}")
    
    """
    =================================================================================================================
                                                Criação dos Plots
    =================================================================================================================
    """
    
    """--------------------------- Criação das Figuras ---------------------------"""
    def create_figures(self):
        # Criar as figuras
        self.fig_positioning = Figure(figsize=(12, 8))
        self.fig_time_evolution = Figure(figsize=(12, 8))
        self.fig_errors = Figure(figsize=(12, 8))
        self.fig_forces = Figure(figsize=(12, 8))

        self.fig_positioning.suptitle("Posicionamento")
        self.fig_time_evolution.suptitle("Evolução Temporal")
        self.fig_errors.suptitle("Erros")
        self.fig_forces.suptitle("Forças|Torques")

        # Criar os Canvas (Widget com Plots)
        self.canvas_positioning = FigureCanvas(self.fig_positioning)
        self.canvas_time_evolution = FigureCanvas(self.fig_time_evolution)
        self.canvas_errors = FigureCanvas(self.fig_errors)
        self.canvas_forces = FigureCanvas(self.fig_forces)
        
        # É necessário adicionar o parent
        self.canvas_positioning.setParent(self)
        self.canvas_time_evolution.setParent(self)
        self.canvas_errors.setParent(self)
        self.canvas_forces.setParent(self)

    """--------------------------- Inicialização dos Dados Temporais ---------------------------"""
    def initialize_data(self):
        self.time_data = []
        self.q_data = [[], [], []]
        self.qd_data = [[], [], []]
        self.qdd_data = [[], [], []]
        self.error_q_data = [[], [], []]
        self.error_qd_data = [[], [], []]
        self.error_qdd_data = [[], [], []]
        self.tau_data = [[], [], []]
    
    """--------------------------- Configuração dos Plots ---------------------------"""
    def setup_plots(self):
        self.setup_positioning_plots()
        self.setup_time_evolution_plots()
        self.setup_errors_plots()
        self.setup_forces_plots()

    """--------------------------- Configuração dos Plots: Positioning Plots ---------------------------"""
    def setup_positioning_plots(self):
        """Configurar os plots de posicionamento"""
        # GridSpec
        gs = GridSpec(nrows=2, ncols=2, left=0.10, right=0.9, top=0.90, bottom=0.08, hspace=0.3, wspace=0.2, width_ratios=[2, 3])
        
        # Plot 3D
        self.ax_3d = self.fig_positioning.add_subplot(gs[:,1], projection='3d')
        self.ax_3d.set(xlim=(-1, 1), ylim=(-1, 1), zlim=(0, 1.5), xlabel='X [m]', ylabel='Y [m]', zlabel='Z [m]', title='Vista 3D')
        
        # Plot XY (vista superior)
        self.ax_xy = self.fig_positioning.add_subplot(gs[0,0])
        self.ax_xy.set(xlim=(-1, 1), ylim=(-1, 1), xlabel='X [m]', ylabel='Y [m]', title='Vista Superior (XY)', aspect=True)
        self.ax_xy.grid(True, alpha=.3)

        # Plot RZ (Vista Lateral)
        self.ax_rz = self.fig_positioning.add_subplot(gs[1,0])
        self.ax_rz.set(xlim=(-0.6, 0.8), ylim=(-0.2, 1.2), xlabel='Distância do centro: R [m]', ylabel='Z [m]', title='Vista Lateral (RZ)', aspect=True)
        self.ax_rz.grid(True, alpha=.3)

    """--------------------------- Configuração dos Plots: Time Evolution Plots ---------------------------"""
    def setup_time_evolution_plots(self):
        """Configurar plots de evolução temporal das juntas"""
        # GridSpec
        gs = GridSpec(nrows=3, ncols=1, left=0.10, right=0.95, top=0.90, bottom=0.08, hspace=0.32)
        
        # Plot de posição q(t)
        self.ax_position = self.fig_time_evolution.add_subplot(gs[0,0])
        self.ax_position.set(xlim=(0,1), ylim=(-1,1), ylabel='Posição', title='Posição das Juntas vs Tempo')
        self.ax_position.grid(True, alpha=.3)
        
        # Plot de velocidade qd(t)
        self.ax_velocity = self.fig_time_evolution.add_subplot(gs[1,0])
        self.ax_velocity.set(xlim=(0,1), ylim=(-1,1), ylabel='Velocidade', title='Velocidade das Juntas vs Tempo')
        self.ax_velocity.grid(True, alpha=.3)
        
        # Plot de aceleração qdd(t)
        self.ax_acceleration = self.fig_time_evolution.add_subplot(gs[2,0])
        self.ax_acceleration.set(xlim=(0,1), ylim=(-1,1), xlabel='Tempo (s)', ylabel='Aceleração', title='Aceleração das Juntas vs Tempo')
        self.ax_acceleration.grid(True, alpha=.3)

    """--------------------------- Configuração dos Plots: Errors Plots ---------------------------"""
    def setup_errors_plots(self):
        """Configurar plots de erros"""
        # GridSpec
        gs = GridSpec(nrows=3, ncols=1, left=0.10, right=0.95, top=0.90, bottom=0.08, hspace=0.32)
        
        # Erro de posição
        self.ax_error_q = self.fig_errors.add_subplot(gs[0,0])
        self.ax_error_q.set(xlim=(0,1), ylim=(-1,1), ylabel='Erro de Posição', title='Erro de Posição vs Tempo')
        self.ax_error_q.grid(True, alpha=.3)
        
        # Erro de velocidade
        self.ax_error_qd = self.fig_errors.add_subplot(gs[1,0])
        self.ax_error_qd.set(xlim=(0,1), ylim=(-1,1), ylabel='Erro de Velocidade', title='Erro de Velocidade vs Tempo')
        self.ax_error_qd.grid(True, alpha=.3)
        
        # Erro de aceleração
        self.ax_error_qdd = self.fig_errors.add_subplot(gs[2,0])
        self.ax_error_qdd.set(xlim=(0,1), ylim=(-1,1), xlabel='Tempo (s)', ylabel='Erro de Aceleração', title='Erro de Aceleração vs Tempo')
        self.ax_error_qdd.grid(True, alpha=.3)

    """--------------------------- Configuração dos Plots: Forces Plots ---------------------------"""
    def setup_forces_plots(self):
        """Configurar plots de Forças|Torques"""
        # GridSpec
        gs = GridSpec(nrows=1, ncols=1, left=0.10, right=0.95, top=0.90, bottom=0.08, hspace=0.32)
        
        # Forças|Torques
        self.ax_forces = self.fig_forces.add_subplot(gs[0,0])
        self.ax_forces.set(xlim=(0,1), ylim=(-1,1), xlabel='Tempo (s)', ylabel='Força|Torque', title='Força|Torque vs Tempo')
        self.ax_forces.grid(True, alpha=.3)

    """--------------------------- Configuração das Linhas ---------------------------"""
    def init_plot_lines(self):
        self.init_positioning_lines()
        self.init_time_evolution_lines()
        self.init_errors_lines()
        self.init_forces_lines()

    """--------------------------- Configuração das Linhas: Positioning Plots ---------------------------"""
    def init_positioning_lines(self):
        """Inicializar linhas dos plots
        - Utiliza-se [] para inicializar as linhas (sem dados)
            - Inicializar significa: setar cor, linha, marker, label, ...
            - Isso é importante pra criar o objeto apenas uma vez e, posteriormente, apenas atualizar o Canvas
        """
        # Robô (pontos azuis interligados de forma contínua)
        self.robot_line_3d, = self.ax_3d.plot([], [], [], 'b-o', linewidth=3, markersize=8, label='Robô')
        self.robot_line_xy, = self.ax_xy.plot([], [], 'b-o', linewidth=3, markersize=8, label='Robô')
        self.robot_line_rz, = self.ax_rz.plot([], [], 'b-o', linewidth=3, markersize=8, label='Robô')

        self.robot_target_line_3d, = self.ax_3d.plot([], [], [], 'g-o', alpha=0.7, linewidth=3, markersize=8, zorder=-1, label='Alvo')
        self.robot_target_line_xy, = self.ax_xy.plot([], [], 'g-o', alpha=0.7, linewidth=3, markersize=8, zorder=-1, label='Alvo')
        self.robot_target_line_rz, = self.ax_rz.plot([], [], 'g-o', alpha=0.7, linewidth=3, markersize=8, zorder=-1, label='Alvo')

        # Trajetória (linha preta pontilhada)
        self.traj_line_3d, = self.ax_3d.plot([], [], [], 'k:', alpha=0.7, zorder=-2, label='Trajetória')
        self.traj_line_xy, = self.ax_xy.plot([], [], 'k:', alpha=0.7, zorder=-2, label='Trajetória')
        self.traj_line_rz, = self.ax_rz.plot([], [], 'k:', alpha=0.7, zorder=-2, label='Trajetória')

        # Linhas de Posicionamento (cartesiano)
        self.x_intial_line, = self.ax_xy.plot([], [], 'b-', alpha=0.5, linewidth=1)
        self.x_final_line, = self.ax_xy.plot([], [], 'g-', alpha=0.5, linewidth=1)
        self.y_intial_line, = self.ax_xy.plot([], [], 'b-', alpha=0.5, linewidth=1)
        self.y_final_line, = self.ax_xy.plot([], [], 'g-', alpha=0.5, linewidth=1)
        self.r_initial_line, = self.ax_rz.plot([], [], 'b.', alpha=0.5, markersize=1)
        self.r_final_line, = self.ax_rz.plot([], [], 'g.', alpha=0.5, markersize=1)

        # Linhas de Posicionamento (juntas)
        center = (0,0)
        self.wedge_q1_initial = Wedge(center, .2, 0, 0, facecolor='b', edgecolor='b', linewidth=1, alpha=0.5)
        self.wedge_q1_final = Wedge(center, .14, 0, 0, facecolor='g', edgecolor='g', linewidth=1, alpha=0.5)
        center = (0,self.robot.L1)
        self.wedge_q2_initial = Wedge(center, .12, -90, 0, facecolor='b', edgecolor='b', linewidth=1, alpha=0.5)
        self.wedge_q2_final = Wedge(center, .08, -90, 0, facecolor='g', edgecolor='g', linewidth=1, alpha=0.5)
        self.d3_initial_line, = self.ax_rz.plot([], [], 'b-', alpha=0.5, linewidth=1)
        self.d3_final_line, = self.ax_rz.plot([], [], 'g-', alpha=0.5, linewidth=1)
        
        # Adicionar legendas
        # self.ax_xy.legend()
        self.ax_rz.legend()
    
    """--------------------------- Configuração das Linhas: Time Evolution Plots ---------------------------"""
    def init_time_evolution_lines(self):
        """Inicializar linhas dos plots de evolução temporal"""
        # Posição
        self.q1_line, = self.ax_position.plot([], [], 'b-', linewidth=2, label=r'$q_1(t)$')
        self.q2_line, = self.ax_position.plot([], [], 'r-', linewidth=2, label=r'$q_2(t)$')
        self.q3_line, = self.ax_position.plot([], [], 'g-', linewidth=2, label=r'$q_3(t)$')
        # Trajetória (plot de posição q(t))
        self.traj_line_q1, = self.ax_position.plot([], [], 'b:', alpha=0.7, label=r'$q_1^{ref}(t)$')
        self.traj_line_q2, = self.ax_position.plot([], [], 'r:', alpha=0.7, label=r'$q_2^{ref}(t)$')
        self.traj_line_q3, = self.ax_position.plot([], [], 'g:', alpha=0.7, label=r'$q_3^{ref}(t)$')
        
        # Velocidade
        self.qd1_line, = self.ax_velocity.plot([], [], 'b-', linewidth=2, label=r'$\dot{q}_1(t)$')
        self.qd2_line, = self.ax_velocity.plot([], [], 'r-', linewidth=2, label=r'$\dot{q}_2(t)$')
        self.qd3_line, = self.ax_velocity.plot([], [], 'g-', linewidth=2, label=r'$\dot{q}_3(t)$')
        # Trajetória (plot de velocidade qd(t))
        self.traj_line_qd1, = self.ax_velocity.plot([], [], 'b:', alpha=0.7, label=r'$\dot{q}_1^{ref}(t)$')
        self.traj_line_qd2, = self.ax_velocity.plot([], [], 'r:', alpha=0.7, label=r'$\dot{q}_2^{ref}(t)$')
        self.traj_line_qd3, = self.ax_velocity.plot([], [], 'g:', alpha=0.7, label=r'$\dot{q}_3^{ref}(t)$')

        # Aceleração
        self.qdd1_line, = self.ax_acceleration.plot([], [], 'b-', linewidth=2, label=r'$\ddot{q}_1(t)$')
        self.qdd2_line, = self.ax_acceleration.plot([], [], 'r-', linewidth=2, label=r'$\ddot{q}_2(t)$')
        self.qdd3_line, = self.ax_acceleration.plot([], [], 'g-', linewidth=2, label=r'$\ddot{q}_3(t)$')
        # Trajetória (plot de aceleração qdd(t))
        self.traj_line_qdd1, = self.ax_acceleration.plot([], [], 'b:', alpha=0.7, label=r'$\ddot{q}_1^{ref}(t)$')
        self.traj_line_qdd2, = self.ax_acceleration.plot([], [], 'r:', alpha=0.7, label=r'$\ddot{q}_2^{ref}(t)$')
        self.traj_line_qdd3, = self.ax_acceleration.plot([], [], 'g:', alpha=0.7, label=r'$\ddot{q}_3^{ref}(t)$')

        # Legendas
        self.ax_position.legend()
        self.ax_velocity.legend()
        self.ax_acceleration.legend()
    
    """--------------------------- Configuração das Linhas: Errors Plots ---------------------------"""
    def init_errors_lines(self):
        """Inicializar linhas dos plots de erro"""
        # Erro de posição
        self.error_q1_line, = self.ax_error_q.plot([], [], 'b-', linewidth=2, label=r'$e_{q_1}(t)$')
        self.error_q2_line, = self.ax_error_q.plot([], [], 'r-', linewidth=2, label=r'$e_{q_2}(t)$')
        self.error_q3_line, = self.ax_error_q.plot([], [], 'g-', linewidth=2, label=r'$e_{q_3}(t)$')
        
        # Erro de velocidade
        self.error_qd1_line, = self.ax_error_qd.plot([], [], 'b-', linewidth=2, label=r'$e_{\dot{q}_1}(t)$')
        self.error_qd2_line, = self.ax_error_qd.plot([], [], 'r-', linewidth=2, label=r'$e_{\dot{q}_2}(t)$')
        self.error_qd3_line, = self.ax_error_qd.plot([], [], 'g-', linewidth=2, label=r'$e_{\dot{q}_3}(t)$')
        
        # Erro de aceleração
        self.error_qdd1_line, = self.ax_error_qdd.plot([], [], 'b-', linewidth=2, label=r'$e_{\ddot{q}_1}(t)$')
        self.error_qdd2_line, = self.ax_error_qdd.plot([], [], 'r-', linewidth=2, label=r'$e_{\ddot{q}_2}(t)$')
        self.error_qdd3_line, = self.ax_error_qdd.plot([], [], 'g-', linewidth=2, label=r'$e_{\ddot{q}_3}(t)$')
        
        # Legendas
        self.ax_error_q.legend()
        self.ax_error_qd.legend()
        self.ax_error_qdd.legend()
    
    """--------------------------- Configuração das Linhas: Forces Plots ---------------------------"""
    def init_forces_lines(self):
        """Inicializar linhas dos plots de Forças|Torques"""
        # Forças e Torques
        self.forces_q1_line, = self.ax_forces.plot([], [], 'b-', linewidth=2, label=r'$\tau_{q_1}(t)$')
        self.forces_q2_line, = self.ax_forces.plot([], [], 'r-', linewidth=2, label=r'$\tau_{q_2}(t)$')
        self.forces_q3_line, = self.ax_forces.plot([], [], 'g-', linewidth=2, label=r'$F_{q_3}(t)$')

        # Legendas
        self.ax_error_q.legend()

    """
    =================================================================================================================
                                                Dados de Referência
    =================================================================================================================
    """

    """--------------------------- Plotar o Workspace ---------------------------"""
    def plot_workspace_views(self):
        """Plotar workspace do robô"""
        try:
            # --------------------------- WORKSPACE RZ --------------------------- #
            # Dados
            r_inner, r_outer = self.robot.r_inner, self.robot.r_outer
            z_inner, z_outer = self.robot.z_inner, self.robot.z_outer

            # Cria polígono fechado para fill():
            r_polygon = np.concatenate([r_outer, r_inner[::-1]])
            z_polygon = np.concatenate([z_outer, z_inner[::-1]])

            # Plotar
            self.ax_rz.plot(r_outer, z_outer, 'g--', label='Reach máximo')
            self.ax_rz.plot(r_inner, z_inner, 'r--', label='Reach mínimo')
            self.ax_rz.fill(r_polygon, z_polygon, alpha=0.2, color='green', label="Workspace")
            self.ax_rz.legend()

            # --------------------------- WORKSPACE XY --------------------------- #            
            # Dados
            x_inner, x_outer = self.robot.x_inner, self.robot.x_outer
            y_inner, y_outer = self.robot.y_inner, self.robot.y_outer

            # Cria polígono fechado para fill():
            x_polygon = np.concatenate([x_outer, x_inner[::-1]])
            y_polygon = np.concatenate([y_outer, y_inner[::-1]])

            # Plotar
            self.ax_xy.plot(x_outer, y_outer, 'g--', alpha=0.5, label='Reach máximo')
            self.ax_xy.plot(x_inner, y_inner, 'r--', alpha=0.5, label='Reach mínimo')
            self.ax_xy.fill(x_polygon, y_polygon, alpha=0.2, color='green', label="Workspace XY")
            # self.ax_xy.legend()

        except Exception as e:
            print(f"Erro ao plotar workspace: {e}")

    """--------------------------- Plotar a Trajetória desejada ---------------------------"""
    def set_trajectory(self, trajectory):
        """Plotar a trajetória desejada"""
        try:
            if trajectory:
                # ------------- Trajetória temporal -------------
                # Separar as variáveis
                t, q_joints, qd_joints, qdd_joints = map(np.array, zip(*trajectory))    # Inteligentíssimo demais, obrigado Claude.AI :)
                q_joints, qd_joints, qdd_joints = q_joints.T, qd_joints.T, qdd_joints.T

                # Atualizar linhas de trajetória
                traj_q_lines = [self.traj_line_q1, self.traj_line_q2, self.traj_line_q3]
                traj_qd_lines = [self.traj_line_qd1, self.traj_line_qd2, self.traj_line_qd3]
                traj_qdd_lines = [self.traj_line_qdd1, self.traj_line_qdd2, self.traj_line_qdd3]
                for i in range(3):
                    traj_q_lines[i].set_data(t, q_joints[i])
                    traj_qd_lines[i].set_data(t, qd_joints[i])
                    traj_qdd_lines[i].set_data(t, qdd_joints[i])
                
                # Conversão de unidades
                # self.traj_line_q1.set_data(t, q1)   # np.rad2deg(q1)
                # self.traj_line_q2.set_data(t, q2)   # np.rad2deg(q2)
                # self.traj_line_q3.set_data(t, q3)   # 1000.0*np.array(q3)

                # ------------- Posição -------------
                # Calcula todas as posições: q -> (x,y,z)
                trajectory_points = []
                for q_joint in q_joints.T:
                    pos = self.robot.forward_kinematics(*q_joint)
                    trajectory_points.append(pos)
                
                x_coords, y_coords, z_coords = np.array(trajectory_points).T
                r_coords = np.sqrt(x_coords**2 + y_coords**2)
                
                self.traj_line_3d.set_data_3d(x_coords, y_coords, z_coords)
                self.traj_line_xy.set_data(x_coords, y_coords)
                self.traj_line_rz.set_data(r_coords, z_coords)

                # Redesenhar
                self.canvas_positioning.draw()
                self.canvas_time_evolution.draw()
                
        except Exception as e:
            print(f"Erro ao atualizar trajetória: {e}")
    
    """--------------------------- Plotar a Posição Alvo (Target) ---------------------------"""
    def set_target_position(self, x, y, z):
        """Definir posição alvo"""
        try:
            r_target = np.sqrt(x**2 + y**2)
            
            self.robot_target_line_xy.set_data([0, x], [0, y])
            self.robot_target_line_rz.set_data([0, r_target], [self.robot.L1, z])
            self.robot_target_line_3d.set_data_3d([0, x], [0, y], [self.robot.L1, z])
            
            self.canvas_positioning.draw()

        except Exception as e:
            print(f"Erro ao definir alvo: {e}")
    
    """--------------------------- Linhas de posicionamento: Cartesiano ---------------------------"""
    def set_cartesian_positionining_lines(self, pos0, posf):
        """Colocar as linhas de posicionamento no modo cartesiano"""
        # Reset
        self.reset_positioning_lines()

        # -------- Colocar as linhas de posicionamento no plot XY --------
        # Dados
        N = 100
        x0, y0, _ = pos0
        xf, yf, _ = posf
        
        # Grid
        grid = np.linspace(-1, 1, N)

        # Plots
        self.x_intial_line.set_data(N*[x0], grid)
        self.y_intial_line.set_data(grid, N*[y0])
        self.x_final_line.set_data(N*[xf], grid)
        self.y_final_line.set_data(grid, N*[yf])

        # -------- Colocar as linhas de posicionamento no plot RZ --------
        r0 = np.sqrt(x0**2 + y0**2)
        rf = np.sqrt(xf**2 + yf**2)
        z0 = self.robot.calc_Zrange(r0)
        zf = self.robot.calc_Zrange(rf)
        # print("z0", z0)

        # Validação: limites físicos
        if z0 is None:      # z0_min
            self.r_initial_line.set_data([], [])
            return
        if zf is None:      # zf_min
            self.r_final_line.set_data([], [])
            return
        
        # Plots
        self.r_initial_line.set_data(len(z0)*[r0], z0)
        self.r_final_line.set_data(len(zf)*[rf], zf)

        self.canvas_positioning.draw()
    
    """--------------------------- Linhas de posicionamento: Juntas ---------------------------"""
    def set_joint_positionining_lines(self, q0, qf):
        # Reset
        self.reset_positioning_lines()
        self.ax_xy.add_patch(self.wedge_q1_initial)
        self.ax_rz.add_patch(self.wedge_q2_initial)
        self.ax_xy.add_patch(self.wedge_q1_final)
        self.ax_rz.add_patch(self.wedge_q2_final)

        # Dados
        q1_0, q2_0, _ = q0
        q1_f, q2_f, _ = qf

        # Parâmetros
        L1, L2, d3_max = self.robot.L1, self.robot.L2, self.robot.d3_max

        # Angulos iniciais
        q1_initial_thetas = (0, np.rad2deg(q1_0)) if q1_0 > 0 else (np.rad2deg(q1_0), 0)
        self.wedge_q1_initial.set_theta1(q1_initial_thetas[0])
        self.wedge_q1_initial.set_theta2(q1_initial_thetas[1])
        
        q2_initial_thetas = (-90, np.rad2deg(q2_0)-90) if q2_0 > 0 else (np.rad2deg(q2_0)-90, -90)
        self.wedge_q2_initial.set_theta1(q2_initial_thetas[0])
        self.wedge_q2_initial.set_theta2(q2_initial_thetas[1])
        
        # Ângulos finais
        q1_final_thetas = (0, np.rad2deg(q1_f)) if q1_f > 0 else (np.rad2deg(q1_f), 0)
        self.wedge_q1_final.set_theta1(q1_final_thetas[0])
        self.wedge_q1_final.set_theta2(q1_final_thetas[1])
        
        q2_final_thetas = (-90, np.rad2deg(q2_f)-90) if q2_f > 0 else (np.rad2deg(q2_f)-90, -90)
        self.wedge_q2_final.set_theta1(q2_final_thetas[0])
        self.wedge_q2_final.set_theta2(q2_final_thetas[1])

        # Comprimento d3
        h = np.array([L2, L2+d3_max])
        r0 = h * np.sin(q2_0)
        rf = h * np.sin(q2_f)
        z0 = L1 - h * np.cos(q2_0)
        zf = L1 - h * np.cos(q2_f)

        self.d3_initial_line.set_data(r0, z0)
        self.d3_final_line.set_data(rf, zf)
        
        self.canvas_positioning.draw()

    """
    =================================================================================================================
                                                Funções de Update
    =================================================================================================================
    """

    """--------------------------- Update da Posição ---------------------------"""
    def update_robot_position(self, q1, q2, d3):
        """Atualizar posição do robô"""
        try:
            # Pega a posição de cada junta, dado (q1, q2, q3)  
            positions = self.robot.get_joint_positions(q1, q2, d3)          
            # Preparar coordenadas
            x_coords, y_coords, z_coords = np.array(positions).T
            # Para vista lateral, usar distância radial (no lugar de x e y)
            r_coords = np.sqrt(x_coords**2 + y_coords**2)
            
            # Atualizar plots
            self.robot_line_3d.set_data_3d(x_coords, y_coords, z_coords)
            self.robot_line_xy.set_data(x_coords, y_coords)
            self.robot_line_rz.set_data(r_coords, z_coords)
            
            # Atualiza a tela
            self.canvas_positioning.draw()
            
        except Exception as e:
            print(f"Erro ao atualizar posição: {e}")
    
    """--------------------------- Update da Posição Temporal ---------------------------"""
    def update_time_evolution(self, t, q, qd, qdd, e, ed, edd, tau):
        """Atualizar gráfico de posição com novos dados"""
        try:
            # Adicionar novos pontos
            self.time_data.append(t)
            for i in range(3):
                self_datas = [self.q_data[i], self.qd_data[i], self.qdd_data[i], self.error_q_data[i], 
                              self.error_qd_data[i], self.error_qdd_data[i], self.tau_data[i]]
                datas = [q[i], qd[i], qdd[i], e[i], ed[i], edd[i], tau[i]]
                for self_data, data in zip(self_datas, datas):
                    self_data.append(data)

            # Conversão de unidade
            # q1_data = np.rad2deg(self.q_data[0])          # rad -> deg
            # q2_data = np.rad2deg(self.q_data[1])          # rad -> deg
            # q3_data = 1000*np.array(self.q_data[2])       # m -> mm
            
            # Atualizar linhas
            q_lines = [self.q1_line, self.q2_line, self.q3_line]
            qd_lines = [self.qd1_line, self.qd2_line, self.qd3_line]
            qdd_lines = [self.qdd1_line, self.qdd2_line, self.qdd3_line]
            e_q_lines = [self.error_q1_line, self.error_q2_line, self.error_q3_line]
            e_qd_lines = [self.error_qd1_line, self.error_qd2_line, self.error_qd3_line]
            e_qdd_lines = [self.error_qdd1_line, self.error_qdd2_line, self.error_qdd3_line]
            tau_lines = [self.forces_q1_line, self.forces_q2_line, self.forces_q3_line]

            for i in range(3):
                lines = [q_lines[i], qd_lines[i], qdd_lines[i], e_q_lines[i], e_qd_lines[i], e_qdd_lines[i], tau_lines[i]]
                datas = [self.q_data[i], self.qd_data[i], self.qdd_data[i], self.error_q_data[i], 
                         self.error_qd_data[i], self.error_qdd_data[i], self.tau_data[i]]
                for line, data in zip(lines, datas):
                    line.set_data(self.time_data, data)
            
            # Ajustar limites automaticamente
            axes = [self.ax_position, self.ax_velocity, self.ax_acceleration]
            datas = [self.q_data, self.qd_data, self.qdd_data]
            e_axes = [self.ax_error_q, self.ax_error_qd, self.ax_error_qdd]
            e_datas = [self.error_q_data, self.error_qd_data, self.error_qdd_data]
            tau_axes = [self.ax_forces]
            tau_datas = [self.tau_data]
            
            # Calcular limites Y
            def get_yrange(data1, data2, data3, margin=1):
                all_data = list(data1) + list(data2) + list(data3)
                all_data = np.concatenate([data1, data2, data3]).flatten().tolist()
                if all_data:
                    y_min = np.min(all_data) - margin
                    y_max = np.max(all_data) + margin
                    return y_min, y_max
            
            # Atualiza os limites do plot
            if self.time_data:
                margins = 3*[.5] + [0.0002, 0.001, 0.001] + [0.05]
                for ax, data, margin in zip(axes+e_axes+tau_axes, datas+e_datas+tau_datas, margins):
                    ax.set_xlim(0, max(self.time_data) + 0.5)
                    ax.set_ylim(*get_yrange(*data, margin=margin))

            # Redesenhar
            self.canvas_time_evolution.draw_idle()
            self.canvas_errors.draw_idle()
            self.canvas_forces.draw_idle()
            
        except Exception as e:
            print(f"Erro ao atualizar gráfico de evolução temporal: {e}")

    """--------------------------- Reset da Posição Temporal ---------------------------"""
    def reset_position_graph(self):
        """Limpar dados do gráfico de Posição X Tempo"""
        try:
            # Limpar dados temporais
            self.initialize_data()

            # Limpar linhas
            q_lines = [self.q1_line, self.q2_line, self.q3_line]
            qd_lines = [self.qd1_line, self.qd2_line, self.qd3_line]
            qdd_lines = [self.qdd1_line, self.qdd2_line, self.qdd3_line]
            e_q_lines = [self.error_q1_line, self.error_q2_line, self.error_q3_line]
            e_qd_lines = [self.error_qd1_line, self.error_qd2_line, self.error_qd3_line]
            e_qdd_lines = [self.error_qdd1_line, self.error_qdd2_line, self.error_qdd3_line]
            tau_lines = [self.forces_q1_line, self.forces_q2_line, self.forces_q3_line]
            for i in range(3):
                lines = [q_lines[i], qd_lines[i], qdd_lines[i], e_q_lines[i], e_qd_lines[i], e_qdd_lines[i], tau_lines[i]]
                for line in lines:
                    line.set_data([], [])

            axes = [self.ax_position, self.ax_velocity, self.ax_acceleration, self.ax_error_q, self.ax_error_qd, self.ax_error_qdd, self.ax_forces]
            for ax in axes:
                ax.set_xlim(0,1)
                ax.set_ylim(-1,1)
            
            # Redesenhar
            self.canvas_time_evolution.draw()
            self.canvas_errors.draw()
            self.canvas_forces.draw()
            
        except Exception as e:
            print(f"Erro ao resetar gráfico de posição: {e}")

    """--------------------------- Reset da Trajetória ---------------------------"""
    def reset_trajectory(self):
        """Limpar dados de trajetória de todos os gráficos"""
        traj_line_q = [self.traj_line_q1, self.traj_line_q2, self.traj_line_q3]
        traj_line_qd = [self.traj_line_qd1, self.traj_line_qd2, self.traj_line_qd3]
        traj_line_qdd = [self.traj_line_qdd1, self.traj_line_qdd2, self.traj_line_qdd3]
        for i in range(3):
            traj_line_q[i].set_data([], [])
            traj_line_qd[i].set_data([], [])
            traj_line_qdd[i].set_data([], [])

        lines = [self.traj_line_xy, self.traj_line_rz, self.robot_target_line_xy, self.robot_target_line_rz]
        for line in lines:
            line.set_data([], [])
        
        self.traj_line_3d.set_data_3d([], [], [])
        self.robot_target_line_3d.set_data_3d([], [], [])

        self.canvas_positioning.draw()

    """--------------------------- Reset das linhas de Posicionamento ---------------------------"""
    def reset_positioning_lines(self):
        """Limpar linhas de posicionamento de todos os gráficos"""
        lines = [self.x_intial_line, self.x_final_line, self.y_intial_line, self.y_final_line, self.r_initial_line, self.r_final_line]
        for line in lines:
            line.set_data([], [])

        try:
            wedges = [self.wedge_q1_initial, self.wedge_q1_final, self.wedge_q2_initial, self.wedge_q2_final]
            for wedge in wedges:
                wedge.remove()
        except:
            pass
        self.d3_initial_line.set_data([], [])
        self.d3_final_line.set_data([], [])

        self.canvas_positioning.draw()
