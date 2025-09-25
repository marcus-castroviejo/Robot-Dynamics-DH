"""
=================================================================================================================
                                                Classe RobotControlInterface 
=================================================================================================================
- Interface principal
    - Toda a definição visual dos paineis e campos da interface
    - Permite a entrada de parâmetros para a simulação: [q_0, q_f, duração, dt, ...] (há validação)
    - Envia dados de "configuração" para a Simulation_Thread: [trajetória desejada, uso de controlador, velocidade da animação]
    - Inicia a simulação na Simulation_Thread através do método simulation_thread.start() -> ativa .run() em simulation_thread.py
    - Recebe sinais da Simulation_Thread "<xyz>_updated" -> ativa funções de update()
        - Essas funções de update() chamam métodos update() em robot_plot.py, que atualizam os plots em tempo real
        - ou alteram diretamente a interface (status/log, barra de progresso, ativar/desativar botões)

Campos deste código:
["Setup Inicial"]:                                  Inicialização, Criação da Interface, Tamanho da Janela
["Criando a User Interface"]:                       Configuração da Interface (Painéis: [1. Controle, 2. Visualização], Campos)
["Métodos Funcionais: Conexão Botões e Sinais"]:    Conexão dos Botões e Sinais da Simulation_Thread com Funções()
["1) Funções de Controle"]:                         Funções ativadas pelos Botões para o Controle da Simulação
["2) Funções de Update (Tempo real)"]:              Funções ativadas pelos Sinais para atualização da interface: update()
["Fechar a Aplicação"]:                             Configuração para finalizar a simulação quando se fecha a interface
"""
import sys
import time
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QGridLayout, QLabel, QLineEdit, 
                            QPushButton, QGroupBox, QSlider, QTextEdit,
                            QFrame, QCheckBox, QMessageBox, QProgressBar, 
                            QSplitter)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QDoubleValidator
from PyQt6.QtCore import QLocale
from dynamics import CocoaBot
from trajectory import TrajectoryGenerator
from visualization import RobotPlot
from .simulation_thread import SimulationThread


class RobotControlInterface(QMainWindow):
    """Interface principal"""
    
    """
    =================================================================================================================
                                                Setup inicial
    =================================================================================================================
    """

    """--------------------------- __init__() ---------------------------"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Controle CocoaBot")
        self.setup_responsive_window()
        
        # Inicializar componentes
        self.init_components()          # Cria os objetos das demais classes (CocoaBot, Trajectory, Simulation_Thread)
        self.init_ui()                  # Cria os painéis de Controle e Visualização
        self.setup_connections()        # Conexão com os Botões (Painel de Controle) e Sinais (Simulation_Thread) -> ativa funções
        
        # Posição inicial padrão
        self.initial_t = 0.0
        self.default_q1 = np.deg2rad(0.0)
        self.default_q2 = np.deg2rad(80.0)
        self.default_d3 = 0.03
        
        # Coloar o robô na posição de início
        self.update_robot_position(self.initial_t, self.default_q1, self.default_q2, self.default_d3)

    """--------------------------- Inicialização das outras Classes ---------------------------"""
    def init_components(self):
        """Inicializar componentes"""
        try:
            self.robot = CocoaBot()
            self.trajectory_gen = TrajectoryGenerator()
            self.simulation_thread = SimulationThread()
            self.trajectory = []
            
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao inicializar componentes: {str(e)}")
    
    """--------------------------- Ajustar o tamanho da Interface ---------------------------"""
    def setup_responsive_window(self):
        """Janela responsiva que se adapta à tela"""
        screen = QApplication.primaryScreen().geometry()
        
        # Usar 90% da tela
        width = int(screen.width() * 0.90)
        height = int(screen.height() * 0.90)

        # Centralizar
        x = (screen.width() - width) // 2
        y = (screen.height() - height) // 2
        
        self.setGeometry(x, y, width, height)

    """
    =================================================================================================================
                                                Criando a User Interface
    =================================================================================================================
    """

    """--------------------------- Criando os Painéis ---------------------------"""
    def init_ui(self):
        """Interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Layout principal com splitter: vai ter Controle e Visualização
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        
        # Painel esquerdo - Controles
        control_panel = self.create_control_panel()
        splitter.addWidget(control_panel)
        
        # Painel direito - Visualização
        viz_panel = self.create_visualization_panel()
        splitter.addWidget(viz_panel)
        
        # Definir proporções
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 3)
        
        main_layout.addWidget(splitter)
    
    """=========================== 1) PAINEL DE CONTROLE ==========================="""
    def create_control_panel(self):
        """Painel de controles"""
        # Layout Geral
        panel = QFrame()
        panel.setFrameStyle(QFrame.Shape.StyledPanel)
        panel.setMinimumWidth(300) # Larguras mínima/máxima
        panel.setMaximumWidth(350)
        layout = QVBoxLayout(panel)
        layout.setSpacing(5) # Espaço entre os Campos
        # layout.setContentsMargins(8, 8, 8, 8) # Margens
        
        # Título                                            # (Linha 0): "Controle CocoaBot"
        title = QLabel("Controle CocoaBot")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("QLabel { color: #2E7D32; padding: 10px; }")
        layout.addWidget(title)
        
        # Adicionando os Campos
        layout.addWidget(self.create_positions_group())     # (Linha 1): Campo de Posição
        layout.addWidget(self.create_trajectory_group())    # (Linha 2): Campo de trajetória
        layout.addWidget(self.create_simulation_group())    # (Linha 3): Campo de Simulação
        layout.addWidget(self.create_status_group())        # (Linha 4): Campo de Status
        
        layout.addStretch()
        return panel
    
    """--------------------------- 1.1) Painel de Controle -> Campo de Posição ---------------------------"""
    def create_positions_group(self):
        """Posições inicial e final"""
        # Layout Geral
        group = QGroupBox("Posições")
        layout = QGridLayout(group)
        layout.setVerticalSpacing(4)
        layout.setHorizontalSpacing(5)
        
        # Validador
        double_validator = QDoubleValidator()
        double_validator.setDecimals(2)
        double_validator.setLocale(QLocale(QLocale.Language.English, QLocale.Country.UnitedStates))

        # Campos de entrada: q_0 e q_f
        self.initial_q1 = QLineEdit("0.0")                  # [entrada] q_0[1]
        self.initial_q2 = QLineEdit("80.0")                 # [entrada] q_0[2]
        self.initial_d3 = QLineEdit("3.0")                  # [entrada] q_0[3]
        self.final_q1 = QLineEdit("90.0")                   # [entrada] q_f[1]
        self.final_q2 = QLineEdit("135.0")                  # [entrada] q_f[2]
        self.final_d3 = QLineEdit("8.0")                    # [entrada] q_f[3]
        # Validador
        self.initial_q1.setValidator(double_validator)
        self.initial_q2.setValidator(double_validator)
        self.initial_d3.setValidator(double_validator)
        self.final_q1.setValidator(double_validator)
        self.final_q2.setValidator(double_validator)
        self.final_d3.setValidator(double_validator)
        # Altura (Height)
        self.initial_q1.setMaximumHeight(35)
        self.initial_q2.setMaximumHeight(35)
        self.initial_d3.setMaximumHeight(35)
        self.final_q1.setMaximumHeight(35)
        self.final_q2.setMaximumHeight(35)
        self.final_d3.setMaximumHeight(35)
        
        """
        OBS - Posicionamento: .addWidget("campo", linha, coluna)
        """
        # Headers: 
        init_label = QLabel("Inicial")              # (Linha 0): " " | "Inicial" | "Final"
        final_label = QLabel("Final")
        layout.addWidget(init_label, 0, 1)
        layout.addWidget(final_label, 0, 2)
        
        # Linhas das juntas: 
        q1_label = QLabel("q1 [deg]:")                # (Linha 1): "q1 [deg]:" | [entrada: initial_q1] | [entrada: final_q1]
        layout.addWidget(q1_label, 1, 0)
        layout.addWidget(self.initial_q1, 1, 1)
        layout.addWidget(self.final_q1, 1, 2)

        q2_label = QLabel("q2 [deg]:")                # (Linha 2): "q2 [deg]:" | [entrada: initial_q2] | [entrada: final_q2]
        layout.addWidget(q2_label, 2, 0)
        layout.addWidget(self.initial_q2, 2, 1)
        layout.addWidget(self.final_q2, 2, 2)

        d3_label = QLabel("d3 [cm]:")               # (Linha 3): "d3 [cm]:" | [entrada: initial_q3] | [entrada: final_q3]
        layout.addWidget(d3_label, 3, 0)
        layout.addWidget(self.initial_d3, 3, 1)
        layout.addWidget(self.final_d3, 3, 2)
        
        return group
    
    """--------------------------- 1.2) Painel de Controle -> Campo de Trajetória ---------------------------"""
    def create_trajectory_group(self):
        """Parâmetros de trajetória"""
        # Layout Geral
        group = QGroupBox("Trajetória")
        layout = QGridLayout(group)
        layout.setVerticalSpacing(4)
        
        # Validador
        double_validator = QDoubleValidator()
        double_validator.setDecimals(0)
        double_validator.setLocale(QLocale(QLocale.Language.English, QLocale.Country.UnitedStates))
        double_validator.setBottom(1)

        # Campos de entrada: Duração e dt
        self.duration = QLineEdit("5.0")        # Duração
        self.dt = QLineEdit("50")               # Dt
        # Validador
        self.duration.setValidator(double_validator)
        self.dt.setValidator(double_validator)
        # Altura
        self.duration.setMaximumHeight(35)
        self.dt.setMaximumHeight(35)
                
        # Adicionar os Widgets
        dur_label = QLabel("Duração [s]:")              # (Linha 0): "Duração [s] | [entrada: duration]"
        layout.addWidget(dur_label, 0, 0)
        layout.addWidget(self.duration, 0, 1)

        dt_label = QLabel("dt [ms]:")                   # (Linha 1): "dt [ms] | [entrada: dt]"
        layout.addWidget(dt_label, 1, 0)
        layout.addWidget(self.dt, 1, 1)
        
        return group
    
    """--------------------------- 1.3) Painel de Controle -> Campo de Simulação ---------------------------"""
    def create_simulation_group(self):
        """Controles de simulação"""
        # Layout Geral
        group = QGroupBox("Simulação")
        layout = QVBoxLayout(group)
        layout.setSpacing(4)
        
        # Checkbox Controlador                                  # (Linha 0): [checkbox] | "Usar controlador Torque Calculado"
        self.use_controller = QCheckBox("Usar controlador Torque Calculado")
        layout.addWidget(self.use_controller)
        
        # Velocidade                                            # (Linha 1): "Velocidade:" | [slider] | "0.6x"
        speed_layout = QHBoxLayout()
        speed_layout.setSpacing(4)
        speed_layout.addWidget(QLabel("Velocidade:"))
        # Slider
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(1, 20)
        self.speed_slider.setValue(10)
        speed_layout.addWidget(self.speed_slider)
        # Label
        self.speed_label = QLabel("1.0x") # --> Velocidade inicial (simulação): 0.6x
        self.speed_label.setFont(QFont("Arial", 11))
        self.speed_label.setMinimumWidth(40)
        speed_layout.addWidget(self.speed_label)
        layout.addLayout(speed_layout)

        # Barra de progresso                                    # (Linha 2): [progress bar] (visível apenas quando começa a simulação)
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)
        
        # Botões                                                # (Linhas 3-6): Botões ("trajetória", "simular", "parar", "robô")
        self.btn_calc_trajectory = QPushButton("Calcular Trajetória")
        self.btn_simulate = QPushButton("Simular")
        self.btn_stop = QPushButton("Parar")
        self.btn_send_robot = QPushButton("Enviar para Robô")
        # Adotar o estilo do botão
        button_style = """
            QPushButton {
                padding: 4px 8px;
                font-size: 11px;
                min-height: 20px;
                max-height: 25px;
            }
        """
        self.btn_calc_trajectory.setStyleSheet(button_style)
        self.btn_simulate.setStyleSheet(button_style)
        self.btn_stop.setStyleSheet(button_style)
        self.btn_send_robot.setStyleSheet(button_style)
        # Alguns botões são desabilitados inicialmente
        self.btn_stop.setEnabled(False)
        self.btn_simulate.setEnabled(False)
        # Adiciona os Widgets
        layout.addWidget(self.btn_calc_trajectory)
        layout.addWidget(self.btn_simulate)
        layout.addWidget(self.btn_stop)
        layout.addWidget(self.btn_send_robot)
        
        return group
    
    """--------------------------- 1.4) Painel de Controle -> Campo de Status (log) ---------------------------"""
    def create_status_group(self):
        """Grupo de status"""
        # Layout Geral
        group = QGroupBox("Status")
        layout = QVBoxLayout(group)
        # layout.setContentsMargins(5, 5, 5, 5)
        
        # Campo de Texto para visualizar o log
        self.status_text = QTextEdit()
        self.status_text.setMaximumHeight(200)
        self.status_text.setReadOnly(True)
        self.status_text.setFont(QFont("Monaco, Consolas, Monospace", 8))
        layout.addWidget(self.status_text)

        return group
    
    """=========================== 2) PAINEL DE VISUALIZAÇÃO ==========================="""
    def create_visualization_panel(self):
        """Painel de visualização"""
        # Layout Geral
        panel = QFrame()
        panel.setFrameStyle(QFrame.Shape.StyledPanel)
        layout = QVBoxLayout(panel)
        # layout.setContentsMargins(5, 5, 5, 5)
        
        # Título                                            # (Linha 0): "Visualização"
        title = QLabel("Visualização")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("QLabel { color: #2E7D32; padding: 10px; }")
        layout.addWidget(title)
        
        # Plots                                             # (Linha 1): [Plots: RobotPlot()]
        try:
            self.robot_plot = RobotPlot(robot=self.robot)
            layout.addWidget(self.robot_plot)
        except Exception as e:
            error_label = QLabel(f"Erro: {str(e)}")
            error_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(error_label)
        
        return panel

    """
    =================================================================================================================
                                                Métodos Funcionais: Conexão Botões e Sinais
    =================================================================================================================
    """

    """--------------------------- Conexão Botões e Sinais ---------------------------"""
    def setup_connections(self):
        """Configurar conexões de sinais"""
        try:
            # Quando os botões são apertados, as funções são acionadas
            self.btn_calc_trajectory.clicked.connect(self.calculate_trajectory)
            self.btn_simulate.clicked.connect(self.start_simulation)
            self.btn_stop.clicked.connect(self.stop_simulation)
            self.btn_send_robot.clicked.connect(self.send_to_robot)
            self.speed_slider.valueChanged.connect(self.update_speed)
            
            # Conecta com a Thread de simulation_thread.py
            # Quando os sinais chegam, as funções de update são acionadas
            self.simulation_thread.position_updated.connect(self.update_robot_position)
            self.simulation_thread.status_updated.connect(self.update_status)
            self.simulation_thread.progress_updated.connect(self.update_progress)
            
        except Exception as e:
            self.update_status(f"Erro ao configurar conexões: {str(e)}")
    
    """
    =================================================================================================================
                                                1) Funções de Controle
    =================================================================================================================
    """

    """--------------------------- 1.1) Funções de Controle -> Calcula Trajetória ---------------------------"""
    def calculate_trajectory(self):
        """Calcular trajetória"""
        try:
            # Validação de Inputs/Entradas
            if not self.validate_inputs():
                return
            
            # [Leitura de dados]: Painel de Controle -> Campo de Posição (q_0 e q_f)
            q0 = np.array([
                np.deg2rad(float(self.initial_q1.text())),
                np.deg2rad(float(self.initial_q2.text())),
                float(self.initial_d3.text()) / 100
            ])
            qf = np.array([
                np.deg2rad(float(self.final_q1.text())),
                np.deg2rad(float(self.final_q2.text())),
                float(self.final_d3.text()) / 100
            ])
            
            # [Leitura de dados]: Painel de Controle -> Campo de Trajetória (Duração e Dt)
            tf = float(self.duration.text())
            dt = float(self.dt.text()) / 1000
            
            # Gera a Trajetória: trajectory = [(t, q_t, qd_t, qdd_t), (...)]
            self.trajectory = self.trajectory_gen.generate_trajectory(q0, qf, tf, dt)
            
            # Calcula todas as posições (x,y,z)
            trajectory_points = []
            for _, q_joints, _, _ in self.trajectory:
                pos = self.robot.forward_kinematics(*q_joints)  # q -> pos(x,y,z)
                trajectory_points.append(pos)

            # [Update plot]: posiciona o robô, adiciona a Trajetória Completa e a Posição Final nos plots
            if hasattr(self, 'robot_plot'):
                self.robot_plot.update_robot_position(*q0)
                self.robot_plot.reset_position_graph()
                self.robot_plot.set_trajectory(trajectory_points)
                x_final, y_final, z_final = self.robot.forward_kinematics(*qf)
                self.robot_plot.set_target_position(x_final, y_final, z_final)
            
            # Ativa o Botão de "Simular"
            self.btn_simulate.setEnabled(True)
            # Adiciona status no log
            self.update_status(f"Trajetória: {len(self.trajectory)} pts, {tf}s")
            
        except ValueError as e:
            QMessageBox.warning(self, "Entrada Inválida", f"Verifique os valores: {str(e)}")
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao calcular trajetória: {str(e)}")
    
    """--------------------------- 1.2) Funções de Controle -> Valida Inputs ---------------------------"""
    def validate_inputs(self):
        """Validar entradas"""
        try:
            fields = [
                (self.initial_q1, "q1 inicial"), (self.initial_q2, "q2 inicial"), (self.initial_d3, "d3 inicial"),
                (self.final_q1, "q1 final"), (self.final_q2, "q2 final"), (self.final_d3, "d3 final"),
                (self.duration, "duração"), (self.dt, "dt")
            ]
            
            for field, name in fields:
                if not field.text().strip():                                                # Validação: entradas nulas
                    QMessageBox.warning(self, "Campo Vazio", f"Preencha '{name}'")
                    field.setFocus()
                    return False
                
                try:                                                                        # Validação: não numéricos
                    float(field.text())
                except ValueError:
                    QMessageBox.warning(self, "Valor Inválido", f"'{name}' deve ser numérico")
                    field.setFocus()
                    return False
            
            # D3: limitante (deve estar dentro dos limites)                     # [Validação 1]: D3 [(0 < D3 < DR_MAX)]
            d3_initial = float(self.initial_d3.text())
            d3_final = float(self.final_d3.text())
            d3_max = 100*self.robot.d3_max
            
            if d3_initial < 0 or d3_initial > d3_max:
                QMessageBox.warning(self, "Limite Físico", f"d3 inicial: 0.0 a {d3_max} cm")
                return False
                
            if d3_final < 0 or d3_final > d3_max:
                QMessageBox.warning(self, "Limite Físico", f"d3 final: 0.0 a {d3_max} cm")
                return False
            
            # Duração: limitante (deve estar dentro dos limites)                # [Validação 2]: Duração e Dt [(Duration > 0) & (0 < dt < duração)]
            duration = float(self.duration.text())
            dt = float(self.dt.text()) / 1000
            
            if duration <= 0:
                QMessageBox.warning(self, "Valor Inválido", "Duração > 0")
                return False
                
            if dt <= 0 or dt >= duration:
                QMessageBox.warning(self, "Valor Inválido", "0 < dt < duração")
                return False
            
            # Posições: limitante (deve estar dentro dos limites)               # [Validação 3]: posição das juntas q_i [RANGE_MIN <= q <= RANGE_MAX] 
            q1_initial = float(self.initial_q1.text())
            q2_initial = float(self.initial_q2.text())
            q1_final = float(self.final_q1.text())
            q2_final = float(self.final_q2.text())
            q1_min, q1_max = np.rad2deg(np.array(self.robot.J1_range, float))
            q2_min, q2_max = np.rad2deg(np.array(self.robot.J2_range, float))

            if q1_initial < q1_min or q1_initial > q1_max:
                QMessageBox.warning(self, "Limite Físico", f"q1 inicial: {q1_min} a {q1_max} graus")
                return False
            
            if q1_final < q1_min or q1_final > q1_max:
                QMessageBox.warning(self, "Limite Físico", f"q1 final: {q1_min} a {q1_max} graus")
                return False
            
            if q2_initial < q2_min or q2_initial > q2_max:
                QMessageBox.warning(self, "Limite Físico", f"q2 inicial: {q2_min} a {q2_max} graus")
                return False
            
            if q2_final < q2_min or q2_final > q2_max:
                QMessageBox.warning(self, "Limite Físico", f"q2 final: {q2_min} a {q2_max} graus")
                return False

            return True
            
        except Exception as e:
            QMessageBox.critical(self, "Erro de Validação", str(e))
            return False
    
    """--------------------------- 1.3) Funções de Controle -> Start Simulação ---------------------------"""
    def start_simulation(self):
        """Iniciar simulação"""
        try:
            if not hasattr(self, 'trajectory') or not self.trajectory:
                QMessageBox.warning(self, "Aviso", "Calcule trajetória primeiro!")
                return
            
            if hasattr(self, 'robot_plot'):
                self.robot_plot.reset_position_graph()
            
            # Passa a trajetória e o controle para o thread                 # Configuração da Simulation_Thread
            self.simulation_thread.set_trajectory(self.trajectory)
            self.simulation_thread.set_controller(self.use_controller.isChecked())
            
            # Ativa/desativa os botões
            self.btn_simulate.setEnabled(False)
            self.btn_stop.setEnabled(True)
            self.btn_calc_trajectory.setEnabled(False)
            self.progress_bar.setVisible(True)
            self.progress_bar.setValue(0)
            
            # Inicia a simulação na thread                                  # Início da Simulation_Thread
            self.simulation_thread.start()
            
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao iniciar simulação: {str(e)}")
            self.reset_simulation_buttons()
    
    """--------------------------- 1.4) Funções de Controle -> Stop Simulação ---------------------------"""
    def stop_simulation(self):
        """Parar simulação"""
        try:
            self.simulation_thread.stop()                                   # Cancelamento da Simulation_Thread
            self.simulation_thread.wait(1000)
            self.update_status("Simulação parada")
            self.reset_simulation_buttons()

        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao parar simulação: {str(e)}")
    
    """--------------------------- 1.5) Funções de Controle -> Reset Botões ---------------------------"""
    def reset_simulation_buttons(self):
        """Resetar botões após simulação"""
        self.btn_simulate.setEnabled(True if hasattr(self, 'trajectory') and self.trajectory else False)
        self.btn_stop.setEnabled(False)
        self.btn_calc_trajectory.setEnabled(True)
        self.progress_bar.setVisible(False)
    
    """--------------------------- 1.6) Funções de Controle -> Enviar para Robô ---------------------------"""
    def send_to_robot(self):
        """Enviar para robô"""
        # Ainda não faz nada. Só printa mensagem.
        try:
            if not hasattr(self, 'trajectory') or not self.trajectory:
                QMessageBox.warning(self, "Aviso", "Calcule trajetória primeiro!")
                return
            
            self.update_status("Preparando envio...")
            QMessageBox.information(self, "Sucesso", "Trajetória enviada!")
            self.update_status("Enviado via Bluetooth")
            
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao enviar: {str(e)}")
    
    """--------------------------- 1.7) Funções de Controle -> Atualizar Velocidade de simulação ---------------------------"""
    def update_speed(self, value):
        """Atualizar velocidade: altera o tempo de sleep() de atualização dos plots"""
        try:
            speed = value / 10.0
            self.speed_label.setText(f"{speed:.1f}x")
            self.simulation_thread.set_speed(speed)
        except Exception as e:
            print(f"Erro ao atualizar velocidade: {e}")
    
    """
    =================================================================================================================
                                                2) Funções de Update (Tempo real)
    =================================================================================================================
    """

    """--------------------------- 2.1) Funções de Update -> Posição e Trajetória das juntas ---------------------------"""
    def update_robot_position(self, t, q1, q2, d3):                             # [sinal]: <- simulation_thread.position_updated
        """Atualizar gráficos de posição"""
        try:
            if hasattr(self, 'robot_plot'):
                self.robot_plot.update_robot_position(q1, q2, d3)               # [update]: -> robot_plot.update_robot_position() [plot: 3d, xy, rz]
                self.robot_plot.update_position_graph(t, q1, q2, d3)            # [update]: -> robot_plot.update_position_graph() [plot: posicao da junta x tempo]
            self.current_t, self.current_q1, self.current_q2, self.current_d3 = t, q1, q2, d3
        except Exception as e:
            print(f"Erro ao atualizar posição: {e}")
    
    """--------------------------- 2.2) Funções de Update -> Status (log) ---------------------------"""
    def update_status(self, message):
        """Atualizar status"""                                                  # [sinal]: <- simulation_thread.status_updated
        try:
            timestamp = time.strftime('%H:%M:%S')
            formatted_message = f"[{timestamp}] {message}"
            self.status_text.append(formatted_message)                          # [update]: -> status_text.append(formatted_message) [adiciona a mensagem no status (log)]
            scrollbar = self.status_text.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
        except Exception as e:
            print(f"Erro ao atualizar status: {e}")
    
    """--------------------------- 2.3) Funções de Update -> Barra de Progresso ---------------------------"""
    def update_progress(self, value):
        """Atualizar progresso"""                                               # [sinal]: <- simulation_thread.progress_updated
        try:
            self.progress_bar.setValue(value)                                   # [update]: -> progress_bar.setValue(value) [atualiza a barra de progresso]
            if value >= 100:
                QTimer.singleShot(1000, self.reset_simulation_buttons)
        except Exception as e:
            print(f"Erro ao atualizar progresso: {e}")
    
    """
    =================================================================================================================
                                                Fechar a Aplicação
    =================================================================================================================
    """

    """--------------------------- Fechar a Aplicação ---------------------------"""
    def closeEvent(self, event):
        """Evento de fechamento"""
        try:
            if self.simulation_thread.isRunning():
                self.simulation_thread.stop()
                self.simulation_thread.wait(2000)
            event.accept()
        except Exception as e:
            print(f"Erro ao fechar aplicação: {e}")
            event.accept()
