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
import matplotlib.gridspec as gridspec
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
            # Tight Layout
            self.fig_positioning.tight_layout()
            self.fig_time_evolution.tight_layout()
            self.fig_errors.tight_layout()
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

        # Criar os Canvas (Widget com Plots)
        self.canvas_positioning = FigureCanvas(self.fig_positioning)
        self.canvas_time_evolution = FigureCanvas(self.fig_time_evolution)
        self.canvas_errors = FigureCanvas(self.fig_errors)
        
        # É necessário adicionar o parent
        self.canvas_positioning.setParent(self)
        self.canvas_time_evolution.setParent(self)
        self.canvas_errors.setParent(self)

        # Tight Layout
        self.fig_positioning.tight_layout()
        self.fig_time_evolution.tight_layout()
        self.fig_errors.tight_layout()

    """--------------------------- Inicialização dos Dados Temporais ---------------------------"""
    def initialize_data(self):
        self.time_data = []
        self.q_data = [[], [], []]
        self.qd_data = [[], [], []]
        self.qdd_data = [[], [], []]
        self.error_q_data = [[], [], []]
        self.error_qd_data = [[], [], []]
        self.error_qdd_data = [[], [], []]
    
    """--------------------------- Configuração dos Plots ---------------------------"""
    def setup_plots(self):
        self.setup_positioning_plots()
        self.setup_time_evolution_plots()
        self.setup_errors_plots()

    """--------------------------- Configuração dos Plots: Positioning Plots ---------------------------"""
    def setup_positioning_plots(self):
        """Configurar os subplots"""
        # Plot 3D principal
        gs = gridspec.GridSpec(nrows=2, ncols=2, width_ratios=[2, 1])
        # self.ax_3d = self.fig_positioning.add_subplot(221, projection='3d')
        self.ax_3d = self.fig_positioning.add_subplot(gs[:,0], projection='3d')
        self.ax_3d.set_xlim(-1, 1)
        self.ax_3d.set_ylim(-1, 1)
        self.ax_3d.set_zlim(0, 1.5)
        self.ax_3d.set_xlabel('X (m)')
        self.ax_3d.set_ylabel('Y (m)')
        self.ax_3d.set_zlabel('Z (m)')
        self.ax_3d.set_title('Vista 3D')
        
        # Plot XY (vista superior)
        # self.ax_xy = self.fig_positioning.add_subplot(222)
        self.ax_xy = self.fig_positioning.add_subplot(gs[0,1])
        self.ax_xy.set_xlim(-1, 1)
        self.ax_xy.set_ylim(-1, 1)
        self.ax_xy.set_xlabel('X (m)')
        self.ax_xy.set_ylabel('Y (m)')
        self.ax_xy.set_title('Vista Superior (XY)')
        self.ax_xy.grid(True, alpha=0.3)
        self.ax_xy.set_aspect('equal')  # deixa os eixos em mesma proporção
        
        # Posição das Juntas (q(t))
        # self.ax_position = self.fig_positioning.add_subplot(223)
        # self.ax_position = self.fig_positioning.add_subplot(gs[1,0])
        # self.ax_position.set_xlim(0, 1)
        # self.ax_position.set_ylim(-5, 165)
        # self.ax_position.set_xlabel('Tempo (s)')
        # self.ax_position.set_ylabel('Posição')
        # self.ax_position.set_title('Posição das Juntas vs Tempo')
        # self.ax_position.grid(True, alpha=0.3)

        # Plot RZ (Vista Lateral)
        # self.ax_rz = self.fig_positioning.add_subplot(224)
        self.ax_rz = self.fig_positioning.add_subplot(gs[1,1])
        self.ax_rz.set_xlim(-.6, .8)
        self.ax_rz.set_ylim(-0.2, 1.2)
        self.ax_rz.set_xlabel('Distância do centro (m)')
        self.ax_rz.set_ylabel('Z (m)')
        self.ax_rz.set_title('Vista Lateral (RZ)')
        self.ax_rz.grid(True, alpha=0.3)
        self.ax_rz.set_aspect('equal')

    """--------------------------- Configuração dos Plots: Time Evolution Plots ---------------------------"""
    def setup_time_evolution_plots(self):
        """Configurar plots de evolução temporal das juntas"""
        # Plot de posição q(t)
        self.ax_position = self.fig_time_evolution.add_subplot(311)
        self.ax_position.set_xlim(0, 1)
        self.ax_position.set_ylim(-5, 165)
        self.ax_position.set_xlabel('Tempo (s)')
        self.ax_position.set_ylabel('Posição')
        self.ax_position.set_title('Posição das Juntas vs Tempo')
        self.ax_position.grid(True, alpha=0.3)
        
        # Plot de velocidade qd(t) - NOVO
        self.ax_velocity = self.fig_time_evolution.add_subplot(312)
        self.ax_velocity.set_xlim(0, 1)
        self.ax_velocity.set_ylim(-50, 50)  # Ajustar conforme necessário
        self.ax_velocity.set_xlabel('Tempo (s)')
        self.ax_velocity.set_ylabel('Velocidade')
        self.ax_velocity.set_title('Velocidade das Juntas vs Tempo')
        self.ax_velocity.grid(True, alpha=0.3)
        
        # Plot de aceleração qdd(t) - NOVO
        self.ax_acceleration = self.fig_time_evolution.add_subplot(313)
        self.ax_acceleration.set_xlim(0, 1)
        self.ax_acceleration.set_ylim(-100, 100)  # Ajustar conforme necessário
        self.ax_acceleration.set_xlabel('Tempo (s)')
        self.ax_acceleration.set_ylabel('Aceleração')
        self.ax_acceleration.set_title('Aceleração das Juntas vs Tempo')
        self.ax_acceleration.grid(True, alpha=0.3)
        # pass

    """--------------------------- Configuração dos Plots: Errors Plots ---------------------------"""
    def setup_errors_plots(self):
        """Configurar plots de erros"""
        # gs = gridspec.GridSpec(nrows=3, ncols=2, width_ratios=[1, 2], height_ratios=[1, 1])
        # Positioning:
        # ax_positioning = self.fig_errors.add_subplot(gs[:,0], projection='3d')
        
        # Erro de posição
        self.ax_error_q = self.fig_errors.add_subplot(311)
        # self.ax_error_q = self.fig_errors.add_subplot(gs[0,1])
        self.ax_error_q.set_xlim(0, 1)
        self.ax_error_q.set_ylim(-10, 10)
        self.ax_error_q.set_xlabel('Tempo (s)')
        self.ax_error_q.set_ylabel('Erro de Posição')
        self.ax_error_q.set_title('Erro de Posição vs Tempo')
        self.ax_error_q.grid(True, alpha=0.3)
        
        # Erro de velocidade
        self.ax_error_qd = self.fig_errors.add_subplot(312)
        # self.ax_error_qd = self.fig_errors.add_subplot(gs[1,1])
        self.ax_error_qd.set_xlim(0, 1)
        self.ax_error_qd.set_ylim(-5, 5)
        self.ax_error_qd.set_xlabel('Tempo (s)')
        self.ax_error_qd.set_ylabel('Erro de Velocidade')
        self.ax_error_qd.set_title('Erro de Velocidade vs Tempo')
        self.ax_error_qd.grid(True, alpha=0.3)
        
        # Erro de aceleração
        self.ax_error_qdd = self.fig_errors.add_subplot(313)
        # self.ax_error_qdd = self.fig_errors.add_subplot(gs[2,1])
        self.ax_error_qdd.set_xlim(0, 1)
        self.ax_error_qdd.set_ylim(-20, 20)
        self.ax_error_qdd.set_xlabel('Tempo (s)')
        self.ax_error_qdd.set_ylabel('Erro de Aceleração')
        self.ax_error_qdd.set_title('Erro de Aceleração vs Tempo')
        self.ax_error_qdd.grid(True, alpha=0.3)

        pass

    """--------------------------- Configuração das Linhas ---------------------------"""
    def init_plot_lines(self):
        self.init_positioning_lines()
        self.init_time_evolution_lines()
        self.init_errors_lines()

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
        
        # Target (ponto verde)
        # self.target_3d, = self.ax_3d.plot([], [], [], 'go', markersize=8, label='Alvo')
        # self.target_xy, = self.ax_xy.plot([], [], 'go', markersize=8, label='Alvo')
        # self.target_rz, = self.ax_rz.plot([], [], 'go', markersize=8, label='Alvo')

        # Trajetória (linha preta pontilhada)
        self.traj_line_3d, = self.ax_3d.plot([], [], [], 'k:', alpha=0.7, label='Trajetória')
        self.traj_line_xy, = self.ax_xy.plot([], [], 'k:', alpha=0.7, label='Trajetória')
        self.traj_line_rz, = self.ax_rz.plot([], [], 'k:', alpha=0.7, label='Trajetória')
        
        # # Trajetória (plot de posição q(t))
        # self.traj_line_q1, = self.ax_position.plot([], [], 'b:', alpha=0.7, label='$q_1^{ref}(t)$ [deg]')
        # self.traj_line_q2, = self.ax_position.plot([], [], 'r:', alpha=0.7, label='$q_2^{ref}(t)$ [deg]')
        # self.traj_line_q3, = self.ax_position.plot([], [], 'g:', alpha=0.7, label='$q_3^{ref}(t)$ [mm]')
        
        # Linhas do Plot das Juntas
        self.q1_line, = self.ax_position.plot([], [], 'b-', linewidth=2, label='$q_1(t)$ [deg]')
        self.q2_line, = self.ax_position.plot([], [], 'r-', linewidth=2, label='$q_2(t)$ [deg]') 
        self.q3_line, = self.ax_position.plot([], [], 'g-', linewidth=2, label='$q_3(t)$ [mm]')

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
        self.ax_position.legend()
    
    """--------------------------- Configuração das Linhas: Time Evolution Plots ---------------------------"""
    def init_time_evolution_lines(self):
        """Inicializar linhas dos plots de evolução temporal"""
        # Posição
        self.q1_line, = self.ax_position.plot([], [], 'b-', linewidth=2, label=r'$q_1(t)$')
        self.q2_line, = self.ax_position.plot([], [], 'r-', linewidth=2, label=r'$q_2(t)$')
        self.q3_line, = self.ax_position.plot([], [], 'g-', linewidth=2, label=r'$q_3(t)$')
        
        # Trajetória (plot de posição q(t))
        self.traj_line_q1, = self.ax_position.plot([], [], 'b:', alpha=0.7, label=r'$q_1^{ref}(t)$ [deg]')
        self.traj_line_q2, = self.ax_position.plot([], [], 'r:', alpha=0.7, label=r'$q_2^{ref}(t)$ [deg]')
        self.traj_line_q3, = self.ax_position.plot([], [], 'g:', alpha=0.7, label=r'$q_3^{ref}(t)$ [mm]')
        
        # Velocidade
        self.qd1_line, = self.ax_velocity.plot([], [], 'b-', linewidth=2, label=r'$\dot{q}_1(t)$')
        self.qd2_line, = self.ax_velocity.plot([], [], 'r-', linewidth=2, label=r'$\dot{q}_2(t)$')
        self.qd3_line, = self.ax_velocity.plot([], [], 'g-', linewidth=2, label=r'$\dot{q}_3(t)$')
        
        # Aceleração
        self.qdd1_line, = self.ax_acceleration.plot([], [], 'b-', linewidth=2, label=r'$\ddot{q}_1(t)$')
        self.qdd2_line, = self.ax_acceleration.plot([], [], 'r-', linewidth=2, label=r'$\ddot{q}_2(t)$')
        self.qdd3_line, = self.ax_acceleration.plot([], [], 'g-', linewidth=2, label=r'$\ddot{q}_3(t)$')

        # Legendas
        self.ax_position.legend()
        self.ax_velocity.legend()
        self.ax_acceleration.legend()

        pass
    
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
    def set_trajectory(self, trajectory, trajectory_points):
        """Plotar a trajetória desejada"""
        try:
            if trajectory and trajectory_points:
                # Trajectory
                t = [ti for ti, _, _, _ in trajectory]
                q_joints = [q_joint for _, q_joint, _, _ in trajectory]
                q1 = [q1 for q1, _, _ in q_joints]
                q2 = [q2 for _, q2, _ in q_joints]
                q3 = [q3 for _, _, q3 in q_joints]

                self.traj_line_q1.set_data(t, np.rad2deg(q1))
                self.traj_line_q2.set_data(t, np.rad2deg(q2))
                self.traj_line_q3.set_data(t, 1000.0*np.array(q3))

                # Position
                x_coords = [point[0] for point in trajectory_points]
                y_coords = [point[1] for point in trajectory_points]
                z_coords = [point[2] for point in trajectory_points]
                r_coords = [np.sqrt(point[0]**2 + point[1]**2) for point in trajectory_points]
                
                self.traj_line_3d.set_data_3d(x_coords, y_coords, z_coords)
                self.traj_line_xy.set_data(x_coords, y_coords)
                self.traj_line_rz.set_data(r_coords, z_coords)
                self.canvas_positioning.draw()
                
        except Exception as e:
            print(f"Erro ao atualizar trajetória: {e}")
    
    """--------------------------- Plotar a Posição Alvo (Target) ---------------------------"""
    def set_target_position(self, x, y, z):
        """Definir posição alvo"""
        try:
            r_target = np.sqrt(x**2 + y**2)
            
            # self.target_3d.set_data_3d([x], [y], [z])
            # self.target_xy.set_data([x], [y])
            # self.target_rz.set_data([r_target], [z])
            
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
    def set_joint_positionining_liens(self, q0, qf):
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
            positions = self.robot.get_joint_positions(q1, q2, d3)  # Pega a posição de cada junta, dado (q1, q2, q3)
            
            # Preparar coordenadas
            x_coords = [pos[0] for pos in positions]
            y_coords = [pos[1] for pos in positions]
            z_coords = [pos[2] for pos in positions]
            
            # Para vista lateral, usar distância radial (no lugar de x e y)
            r_coords = [np.sqrt(pos[0]**2 + pos[1]**2) for pos in positions]
            
            # Atualizar plots
            self.robot_line_3d.set_data_3d(x_coords, y_coords, z_coords)
            self.robot_line_xy.set_data(x_coords, y_coords)
            self.robot_line_rz.set_data(r_coords, z_coords)
            
            # Atualiza a tela
            self.canvas_positioning.draw()
            
        except Exception as e:
            print(f"Erro ao atualizar posição: {e}")
    
    """--------------------------- Update da Posição Temporal ---------------------------"""
    def update_position_graph(self, t, q1, q2, d3):
        """Atualizar gráfico de posição com novos dados"""
        try:
            # Adicionar novos pontos
            self.time_data.append(t)
            self.q_data[0].append(q1)
            self.q_data[1].append(q2)
            self.q_data[2].append(d3)
            # self.q1_data.append(q1)
            # self.q2_data.append(q2)
            # self.q3_data.append(d3)

            # Conversão de unidade
            q1_data = np.rad2deg(self.q_data[0])          # rad -> deg
            q2_data = np.rad2deg(self.q_data[1])          # rad -> deg
            q3_data = 1000*np.array(self.q_data[2])       # m -> mm
            
            # Atualizar linhas
            self.q1_line.set_data(self.time_data, q1_data)
            self.q2_line.set_data(self.time_data, q2_data)
            self.q3_line.set_data(self.time_data, q3_data)
            
            # Ajustar limites automaticamente
            if self.time_data:
                self.ax_position.set_xlim(0, max(self.time_data) + 0.5)
                
                # Calcular limites Y
                all_data = list(q1_data) + list(q2_data) + list(q3_data)
                if all_data:
                    margin = 5
                    y_min = min(all_data) - margin
                    y_max = max(all_data) + margin
                    self.ax_position.set_ylim(y_min, y_max)
            
            # Redesenhar
            # self.ax_position.figure.canvas.draw_idle()
            # self.canvas_positioning.draw_idle()
            self.canvas_time_evolution.draw_idle()
            
        except Exception as e:
            print(f"Erro ao atualizar gráfico de posição: {e}")

    """--------------------------- Reset da Posição Temporal ---------------------------"""
    def reset_position_graph(self):
        """Limpar dados do gráfico de Posição X Tempo"""
        try:
            # self.time_data.clear()
            # self.q_data.clear()
            # self.q1_data.clear()
            # self.q2_data.clear()
            # self.q3_data.clear()
            self.initialize_data()

            # Limpar linhas
            self.q1_line.set_data([], [])
            self.q2_line.set_data([], [])
            self.q3_line.set_data([], [])
            self.ax_position.set_xlim(0, 1)
            self.ax_position.set_ylim(-5, 165)
            
            # self.canvas_positioning.draw()
            self.canvas_time_evolution.draw()
            
        except Exception as e:
            print(f"Erro ao resetar gráfico de posição: {e}")

    """--------------------------- Reset da Trajetória ---------------------------"""
    def reset_trajectory(self):
        """Limpar dados de trajetória de todos os gráficos"""
        self.traj_line_q1.set_data([], [])
        self.traj_line_q2.set_data([], [])
        self.traj_line_q3.set_data([], [])
        self.traj_line_3d.set_data_3d([], [], [])
        self.traj_line_xy.set_data([], [])
        self.traj_line_rz.set_data([], [])
        # self.target_3d.set_data_3d([], [], [])
        # self.target_xy.set_data([], [])
        # self.target_rz.set_data([], [])
        self.robot_target_line_3d.set_data_3d([], [], [])
        self.robot_target_line_xy.set_data([], [])
        self.robot_target_line_rz.set_data([], [])

        self.canvas_positioning.draw()

    """--------------------------- Reset das linhas de Posicionamento ---------------------------"""
    def reset_positioning_lines(self):
        """Limpar linhas de posicionamento de todos os gráficos"""
        self.x_intial_line.set_data([], [])
        self.y_intial_line.set_data([], [])
        self.x_final_line.set_data([], [])
        self.y_final_line.set_data([], [])
        self.r_initial_line.set_data([], [])
        self.r_final_line.set_data([], [])

        try:
            self.wedge_q1_initial.remove()
            self.wedge_q2_initial.remove()
            self.wedge_q1_final.remove()
            self.wedge_q2_final.remove()
        except:
            pass
        self.d3_initial_line.set_data([], [])
        self.d3_final_line.set_data([], [])

        self.canvas_positioning.draw()
