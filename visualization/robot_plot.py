"""
============================================
    Classe RobotPlot3D
============================================
- Gráficos do Cocoabot

É aqui que todos os gráficos da interface são gerados:
- 4 plots: 3D, vista XY, vista lateral, workspace
- Esqueleto do robô, atualizac1ão em tempo real...

"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt6.QtWidgets import QWidget
from dynamics import CocoaBot


class RobotPlot3D(FigureCanvas):
    """Visualização corrigida para robô RRP"""
    
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(12, 8))
        super().__init__(self.fig)
        self.setParent(parent)
        
        try:
            # Configurar subplots
            self.setup_plots()              # Configura os subplots: labels, limits, title, grid, ...
            self.robot = CocoaBot()         # Cria o Cocoabot
            self.trajectory_points = []     # Declara trajectory_points
            
            # Inicializar linhas
            self.init_plot_lines()          # Inicializa as linhas do Robô, Trajetória e Alvo
            self.plot_workspace_views()     
            
            self.fig.tight_layout()
            
        except Exception as e:
            print(f"Erro ao inicializar plot: {e}")
    
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
        self.ax_3d.set_title('Cocoabot - Vista 3D')
        
        # Plot XY (vista superior)
        self.ax_xy = self.fig.add_subplot(222)
        self.ax_xy.set_xlim(-1, 1)
        self.ax_xy.set_ylim(-1, 1)
        self.ax_xy.set_xlabel('X (m)')
        self.ax_xy.set_ylabel('Y (m)')
        self.ax_xy.set_title('Vista Superior (XY)')
        self.ax_xy.grid(True)
        self.ax_xy.set_aspect('equal')  # deixa os eixos em mesma proporção
        
        # Posição das Juntas (q)
        self.ax_position = self.fig.add_subplot(223)
        self.plot_position_graph()

        # Plot RZ (Vista Lateral)
        self.ax_rz = self.fig.add_subplot(224)
        self.ax_rz.set_xlim(-.8, .8)
        self.ax_rz.set_ylim(-0.2, 1.2)
        self.ax_rz.set_xlabel('Distância do centro (m)')
        self.ax_rz.set_ylabel('Z (m)')
        self.ax_rz.set_title('Vista Lateral (RZ)')
        self.ax_rz.grid(True)
        self.ax_rz.set_aspect('equal')  # deixa os eixos em mesma proporção
        
    
    def init_plot_lines(self):
        """Inicializar linhas dos plots"""
        """
        - Utiliza-se [] para inicializar as linhas, mesmo sem dados
            - Inicializar significa: setar cor, linha, marker, label, ...
            - Isso é importante pra criar o objeto apenas uma vez e, posteriormente,
              apenas atualizar...
        - 
        """
        # Linhas do robô (pontos azul interligados de forma contínua)
        self.robot_line_3d, = self.ax_3d.plot([], [], [], 'b-o', linewidth=3, markersize=8, label='Robô')
        self.robot_line_xy, = self.ax_xy.plot([], [], 'b-o', linewidth=3, markersize=8, label='Robô')
        self.robot_line_rz, = self.ax_rz.plot([], [], 'b-o', linewidth=3, markersize=8, label='Robô')
        
        # Trajetória (linha preta pontilhada)
        self.traj_line_3d, = self.ax_3d.plot([], [], [], 'k:', alpha=0.7, label='Trajetória')
        self.traj_line_xy, = self.ax_xy.plot([], [], 'k:', alpha=0.7, label='Trajetória')
        self.traj_line_rz, = self.ax_rz.plot([], [], 'k:', alpha=0.7, label='Trajetória')
        
        # Target (ponto verde)
        self.target_3d, = self.ax_3d.plot([], [], [], 'go', markersize=10, label='Alvo')
        self.target_xy, = self.ax_xy.plot([], [], 'go', markersize=10, label='Alvo')
        self.target_rz, = self.ax_rz.plot([], [], 'go', markersize=10, label='Alvo')
        
        # Adicionar legendas
        self.ax_xy.legend()
        self.ax_rz.legend()
    
    def plot_workspace_views(self):
        """Plotar workspace do robô RRP"""
        try:
            L1 = self.robot.L1
            L2 = self.robot.L2
            q1_range = self.robot.J1_range
            q2_range = self.robot.J2_range
            d3_max = self.robot.d3_max
            
            # === WORKSPACE YZ ===
            # Alcance radial no plano XY
            q2 = np.linspace(*q2_range, 100)
            r_inner = np.min(np.sin(q2) * L2)
            r_outer = np.max(np.sin(q2) * (L2 + d3_max))

            # Varredura em q1 (z)
            q1 = np.linspace(*q1_range, 100)
            x_inner = r_inner * np.cos(q1)
            y_inner = r_inner * np.sin(q1)
            x_outer = r_outer * np.cos(q1)
            y_outer = r_outer * np.sin(q1)
            
            # Plotar
            self.ax_xy.plot(x_outer, y_outer, 'g--', alpha=0.5, label='Reach máximo')
            self.ax_xy.plot(x_inner, y_inner, 'r--', alpha=0.5, label='Reach mínimo')
            self.ax_xy.fill(x_inner, y_inner, color='red', alpha=0.2, label='Inacessível')
            self.ax_xy.legend()

            # === WORKSPACE RZ ===
            q2 = q2 - np.pi/4                   # (pi/4 ???)

            # Raio interno e externo
            r_inner = L2
            r_outer = L2 + d3_max
            
            # Valores assumidos
            r_min = np.array(r_inner * np.sin(q2))
            z_min = np.array(L1 + r_inner * np.cos(q2))
            r_max = np.array(r_outer * np.sin(q2))
            z_max = np.array(L1 + r_outer * np.cos(q2))

            # Plotar contornos com suas labels originais
            self.ax_rz.plot(r_max, z_max, 'g--', label='Reach máximo')
            self.ax_rz.plot(r_min, z_min, 'r--', label='Reach mínimo')

            # Cria polígono fechado para fill():
            r_polygon = np.concatenate([r_max, r_min[::-1]])
            z_polygon = np.concatenate([z_max, z_min[::-1]])

            # Preencher área (sem label para evitar duplicação na legenda)
            self.ax_rz.fill(r_polygon, z_polygon, alpha=0.2, color='green', label="Workspace YZ")
            self.ax_rz.legend()
            
        except Exception as e:
            print(f"Erro ao plotar workspace: {e}")
    
    def plot_position_graph(self):
        """Configurar gráfico de posição das juntas"""
        try:
            # Inicializar arrays
            self.time_data = []
            self.q1_data = []
            self.q2_data = []
            self.q3_data = []
            
            # TODA A CONFIGURAÇÃO AQUI:
            self.ax_position.set_xlim(0, 5)
            self.ax_position.set_ylim(-0.5, 3.5)
            self.ax_position.set_xlabel('Tempo (s)')
            self.ax_position.set_ylabel('Posição')
            self.ax_position.set_title('Posição das Juntas vs Tempo')
            self.ax_position.grid(True, alpha=0.3)
            
            # Criar linhas
            self.q1_line, = self.ax_position.plot([], [], 'b-', linewidth=2, label='q1(t) [rad]')
            self.q2_line, = self.ax_position.plot([], [], 'r-', linewidth=2, label='q2(t) [rad]') 
            self.q3_line, = self.ax_position.plot([], [], 'g-', linewidth=2, label='q3(t) [m]')
            
            self.ax_position.legend()
            
        except Exception as e:
            print(f"Erro ao configurar gráfico de posição: {e}")

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
    
    def update_trajectory(self, trajectory_points):
        """Atualizar trajetória"""
        try:
            if trajectory_points:
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

    def update_position_graph(self, t, q1, q2, d3):
        """Atualizar gráfico de posição com novos dados"""
        try:
            # Adicionar novos pontos
            self.time_data.append(t)
            self.q1_data.append(q1)
            self.q2_data.append(q2)
            self.q3_data.append(d3)
            
            # Atualizar linhas
            self.q1_line.set_data(self.time_data, self.q1_data)
            self.q2_line.set_data(self.time_data, self.q2_data)
            self.q3_line.set_data(self.time_data, self.q3_data)
            
            # Ajustar limites automaticamente
            if self.time_data:
                self.ax_position.set_xlim(0, max(self.time_data) + 0.5)
                
                # Calcular limites Y baseado nos dados
                all_data = self.q1_data + self.q2_data + self.q3_data
                if all_data:
                    margin = 0.1
                    y_min = min(all_data) - margin
                    y_max = max(all_data) + margin
                    self.ax_position.set_ylim(y_min, y_max)
            
            # Redesenhar
            self.ax_position.figure.canvas.draw_idle()
            
        except Exception as e:
            print(f"Erro ao atualizar gráfico de posição: {e}")

    def reset_position_graph(self):
        """Limpar dados do gráfico de posição"""
        try:
            self.time_data.clear()
            self.q1_data.clear()
            self.q2_data.clear()
            self.q3_data.clear()
            
            # Limpar linhas
            self.q1_line.set_data([], [])
            self.q2_line.set_data([], [])
            self.q3_line.set_data([], [])
            
            self.draw()
            
        except Exception as e:
            print(f"Erro ao resetar gráfico de posição: {e}")