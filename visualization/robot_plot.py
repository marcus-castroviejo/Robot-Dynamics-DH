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
from PyQt6.QtWidgets import QWidget


class RobotPlot(FigureCanvas):
    """Visualização para a Interface"""
    
    """
    =================================================================================================================
                                                Setup inicial
    =================================================================================================================
    """

    """--------------------------- __init__() ---------------------------"""
    def __init__(self, robot, parent=None):
        self.fig = Figure(figsize=(12, 8))
        super().__init__(self.fig)
        self.setParent(parent)

        # Salvar dados para Posição das Juntas: q(t)
        self.time_data = []
        self.q1_data = []
        self.q2_data = []
        self.q3_data = []
        
        try:
            # Configurar plots
            self.setup_plots()              # Configura: posição, title, labels, limits, grid, ...
            self.robot = robot
            self.trajectory_points = []
            
            # Inicializar linhas
            self.init_plot_lines()          # Inicializa as linhas dos plots
            self.plot_workspace_views()     # Desenha o Workspace
            
            self.fig.tight_layout()
            
        except Exception as e:
            print(f"Erro ao inicializar plot: {e}")
    
    """
    =================================================================================================================
                                                Criação dos Plots
    =================================================================================================================
    """
    
    """--------------------------- Configuração dos Plots ---------------------------"""
    def setup_plots(self):
        """Configurar os subplots"""
        # Plot 3D principal
        self.ax_3d = self.fig.add_subplot(221, projection='3d')
        self.ax_3d.set_xlim(-1, 1)
        self.ax_3d.set_ylim(-1, 1)
        self.ax_3d.set_zlim(0, 1.5)
        self.ax_3d.set_xlabel('X (m)')
        self.ax_3d.set_ylabel('Y (m)')
        self.ax_3d.set_zlabel('Z (m)')
        self.ax_3d.set_title('Vista 3D')
        
        # Plot XY (vista superior)
        self.ax_xy = self.fig.add_subplot(222)
        self.ax_xy.set_xlim(-1, 1)
        self.ax_xy.set_ylim(-1, 1)
        self.ax_xy.set_xlabel('X (m)')
        self.ax_xy.set_ylabel('Y (m)')
        self.ax_xy.set_title('Vista Superior (XY)')
        self.ax_xy.grid(True, alpha=0.3)
        self.ax_xy.set_aspect('equal')  # deixa os eixos em mesma proporção
        
        # Posição das Juntas (q(t))
        self.ax_position = self.fig.add_subplot(223)
        self.ax_position.set_xlim(0, 1)
        self.ax_position.set_ylim(-5, 165)
        self.ax_position.set_xlabel('Tempo (s)')
        self.ax_position.set_ylabel('Posição')
        self.ax_position.set_title('Posição das Juntas vs Tempo')
        self.ax_position.grid(True, alpha=0.3)

        # Plot RZ (Vista Lateral)
        self.ax_rz = self.fig.add_subplot(224)
        self.ax_rz.set_xlim(-.8, .8)
        self.ax_rz.set_ylim(-0.2, 1.2)
        self.ax_rz.set_xlabel('Distância do centro (m)')
        self.ax_rz.set_ylabel('Z (m)')
        self.ax_rz.set_title('Vista Lateral (RZ)')
        self.ax_rz.grid(True, alpha=0.3)
        self.ax_rz.set_aspect('equal')
        
    """--------------------------- Configuração das Linhas ---------------------------"""
    def init_plot_lines(self):
        """Inicializar linhas dos plots
        - Utiliza-se [] para inicializar as linhas (sem dados)
            - Inicializar significa: setar cor, linha, marker, label, ...
            - Isso é importante pra criar o objeto apenas uma vez e, posteriormente, apenas atualizar o Canvas
        """
        # Robô (pontos azuis interligados de forma contínua)
        self.robot_line_3d, = self.ax_3d.plot([], [], [], 'b-o', linewidth=3, markersize=8, label='Robô')
        self.robot_line_xy, = self.ax_xy.plot([], [], 'b-o', linewidth=3, markersize=8, label='Robô')
        self.robot_line_rz, = self.ax_rz.plot([], [], 'b-o', linewidth=3, markersize=8, label='Robô')
        
        # Trajetória (linha preta pontilhada)
        self.traj_line_3d, = self.ax_3d.plot([], [], [], 'k:', alpha=0.7, label='Trajetória')
        self.traj_line_xy, = self.ax_xy.plot([], [], 'k:', alpha=0.7, label='Trajetória')
        self.traj_line_rz, = self.ax_rz.plot([], [], 'k:', alpha=0.7, label='Trajetória')
        
        # Trajetória (plot de posição q(t))
        self.traj_line_q1, = self.ax_position.plot([], [], 'b:', alpha=0.7, label='$q_1^{ref}(t)$ [deg]')
        self.traj_line_q2, = self.ax_position.plot([], [], 'r:', alpha=0.7, label='$q_2^{ref}(t)$ [deg]')
        self.traj_line_q3, = self.ax_position.plot([], [], 'g:', alpha=0.7, label='$q_3^{ref}(t)$ [mm]')
        
        # Target (ponto verde)
        self.target_3d, = self.ax_3d.plot([], [], [], 'go', markersize=8, label='Alvo')
        self.target_xy, = self.ax_xy.plot([], [], 'go', markersize=8, label='Alvo')
        self.target_rz, = self.ax_rz.plot([], [], 'go', markersize=8, label='Alvo')
        
        # Linhas do Plot das Juntas
        self.q1_line, = self.ax_position.plot([], [], 'b-', linewidth=2, label='$q_1(t)$ [deg]')
        self.q2_line, = self.ax_position.plot([], [], 'r-', linewidth=2, label='$q_2(t)$ [deg]') 
        self.q3_line, = self.ax_position.plot([], [], 'g-', linewidth=2, label='$q_3(t)$ [mm]')

        # Linhas de Posicionamento
        self.x_intial_line, = self.ax_xy.plot([], [], 'b-', alpha=0.5, linewidth=1)
        self.x_final_line, = self.ax_xy.plot([], [], 'g-', alpha=0.5, linewidth=1)
        self.y_intial_line, = self.ax_xy.plot([], [], 'b-', alpha=0.5, linewidth=1)
        self.y_final_line, = self.ax_xy.plot([], [], 'g-', alpha=0.5, linewidth=1)
        self.r_initial_line, = self.ax_rz.plot([], [], 'b-', alpha=0.5, linewidth=1)
        self.r_final_line, = self.ax_rz.plot([], [], 'g-', alpha=0.5, linewidth=1)
        
        # Adicionar legendas
        # self.ax_xy.legend()
        self.ax_rz.legend()
        self.ax_position.legend()
    
    """
    =================================================================================================================
                                                Dados de Referência
    =================================================================================================================
    """

    """--------------------------- Plotar o Workspace ---------------------------"""
    def plot_workspace_views(self):
        """Plotar workspace do robô"""
        try:
            # Dimensões e limites de movimentação
            L1 = self.robot.L1
            L2 = self.robot.L2
            q1_range = self.robot.J1_range
            q2_range = self.robot.J2_range
            d3_max = self.robot.d3_max

            # Ranges de movimentação
            q1 = np.linspace(*q1_range, 100)
            q2 = np.linspace(*q2_range, 100)

            # Hipotenusa mínima e máxima (Elo 2)
            h_min = L2
            h_max = L2 + d3_max
            
            # Coordenada radial e coordenada Z
            r_inner = np.array(h_min * np.sin(q2))
            r_outer = np.array(h_max * np.sin(q2))
            z_inner = np.array(L1 - h_min * np.cos(q2))
            z_outer = np.array(L1 - h_max * np.cos(q2))

            # --------------------------- WORKSPACE RZ --------------------------- #
            # Cria polígono fechado para fill():
            r_polygon = np.concatenate([r_outer, r_inner[::-1]])
            z_polygon = np.concatenate([z_outer, z_inner[::-1]])

            # Plotar
            self.ax_rz.plot(r_outer, z_outer, 'g--', label='Reach máximo')
            self.ax_rz.plot(r_inner, z_inner, 'r--', label='Reach mínimo')
            self.ax_rz.fill(r_polygon, z_polygon, alpha=0.2, color='green', label="Workspace RZ")
            self.ax_rz.legend()

            # --------------------------- WORKSPACE XY --------------------------- #
            # Alcance radial mínimo e máximo no plano XY
            r_min = np.min(r_inner)
            r_max = np.max(r_outer)

            # Coordenas X e Y
            x_inner = r_min * np.cos(q1)
            x_outer = r_max * np.cos(q1)
            y_inner = r_min * np.sin(q1)
            y_outer = r_max * np.sin(q1)
            
            # Plotar
            self.ax_xy.plot(x_outer, y_outer, 'g--', alpha=0.5, label='Reach máximo')
            self.ax_xy.plot(x_inner, y_inner, 'r--', alpha=0.5, label='Reach mínimo')
            x_polygon = np.concatenate([x_outer, x_inner[::-1]])
            y_polygon = np.concatenate([y_outer, y_inner[::-1]])
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
                self.draw()
                
        except Exception as e:
            print(f"Erro ao atualizar trajetória: {e}")
    
    """--------------------------- Plotar a Posição Alvo (Target) ---------------------------"""
    def set_target_position(self, x, y, z):
        """Definir posição alvo"""
        try:
            r_target = np.sqrt(x**2 + y**2)
            
            self.target_3d.set_data_3d([x], [y], [z])
            self.target_xy.set_data([x], [y])
            self.target_rz.set_data([r_target], [z])
            self.draw()
            
        except Exception as e:
            print(f"Erro ao definir alvo: {e}")
    
    """--------------------------- Linhas de posicionamento: plot XY ---------------------------"""
    def set_xy_pos(self, pos0, posf):
        """Colocar as linhas de posicionamento no plot XY"""
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

        self.draw()
    
    """--------------------------- Linhas de Posicionamento: plot RZ ---------------------------"""
    def set_rz_pos(self, pos0, posf):
        """Colocar as linhas de posicionamento no plot RZ"""
        # Função para o range de Z
        def Z_range(r):
            # Parâmetros
            L1 = self.robot.L1
            L2 = self.robot.L2
            d3_max = self.robot.d3_max
            q2_range = self.robot.J2_range

            q2 = np.linspace(*q2_range, 200)
            sin_q2 = np.sin(q2)
            valid_sin = np.abs(sin_q2) > 1e-6
            d3 = np.where(valid_sin, r / sin_q2 - L2, np.inf)
            valid = valid_sin & (d3 >= 0) & (d3 <= d3_max)
            if not np.any(valid):
                return None, None
            z = L1 - (L2 + d3[valid]) * np.cos(q2[valid])

            return np.min(z), np.max(z)

        # Cálculo
        x0, y0, _ = pos0
        xf, yf, _ = posf
        r0 = np.sqrt(x0**2 + y0**2)
        rf = np.sqrt(xf**2 + yf**2)
        z0_min, z0_max = Z_range(r0)
        zf_min, zf_max = Z_range(rf)

        if z0_min is None:
            self.r_initial_line.set_data([], [])
            return
        if zf_min is None:
            self.r_final_line.set_data([], [])
            return
       
        # Grids
        N = 100
        z0_grid = np.linspace(z0_min, z0_max, N)
        zf_grid = np.linspace(zf_min, zf_max, N)
        
        # Plot R
        self.r_initial_line.set_data(N*[r0], z0_grid)
        self.r_final_line.set_data(N*[rf], zf_grid)

        self.draw()

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
            self.draw()
            
        except Exception as e:
            print(f"Erro ao atualizar posição: {e}")
    
    """--------------------------- Update da Posição Temporal ---------------------------"""
    def update_position_graph(self, t, q1, q2, d3):
        """Atualizar gráfico de posição com novos dados"""
        try:
            # Adicionar novos pontos
            self.time_data.append(t)
            self.q1_data.append(q1)
            self.q2_data.append(q2)
            self.q3_data.append(d3)

            # Conversão de unidade
            q1_data = np.rad2deg(self.q1_data)          # rad -> deg
            q2_data = np.rad2deg(self.q2_data)          # rad -> deg
            q3_data = 1000*np.array(self.q3_data)       # m -> mm
            
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
            self.ax_position.figure.canvas.draw_idle()
            
        except Exception as e:
            print(f"Erro ao atualizar gráfico de posição: {e}")

    """--------------------------- Reset da Posição Temporal ---------------------------"""
    def reset_position_graph(self):
        """Limpar dados do gráfico de Posição X Tempo"""
        try:
            self.time_data.clear()
            self.q1_data.clear()
            self.q2_data.clear()
            self.q3_data.clear()
            
            # Limpar linhas
            self.q1_line.set_data([], [])
            self.q2_line.set_data([], [])
            self.q3_line.set_data([], [])
            self.ax_position.set_xlim(0, 1)
            self.ax_position.set_ylim(-5, 165)
            
            self.draw()
            
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
        self.target_3d.set_data_3d([], [], [])
        self.target_xy.set_data([], [])
        self.target_rz.set_data([], [])

        self.draw()

    """--------------------------- Reset das linhas de Posicionamento ---------------------------"""
    def reset_positioning_lines(self):
        """Limpar linhas de posicionamento de todos os gráficos"""
        self.x_intial_line.set_data([], [])
        self.y_intial_line.set_data([], [])
        self.x_final_line.set_data([], [])
        self.y_final_line.set_data([], [])
        self.r_initial_line.set_data([], [])
        self.r_final_line.set_data([], [])

        self.draw()
