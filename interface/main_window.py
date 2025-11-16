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
["0) Funções Auxiliares"]:                          Leitura de dados, Validação, Ativação de botões, Limpar interface
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
                            QSplitter, QRadioButton, QTabWidget)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QDoubleValidator
from PyQt6.QtCore import QLocale, QSignalBlocker
from dynamics import CocoaBot, CalculatedTorqueController
from trajectory import TrajectoryGenerator
from visualization import RobotPlot
from .simulation_thread import SimulationThread
from esp32_comm import ESP32Server



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
        
        # Estilo para os botões
        self.error_style = """
        QLineEdit {
            border: 2px solid #dc3545;
            background-color: #fff5f5;
        }"""
        self.normal_style = ""

        # Inicializar componentes
        self.init_components()          # Cria os objetos das demais classes (CocoaBot, Trajectory, Simulation_Thread)
        self.init_ui()                  # Cria os painéis de Controle e Visualização
        self.setup_connections()        # Conexão com os Botões (Painel de Controle) e Sinais (Simulation_Thread) -> ativam funções
        
        # Posição inicial padrão
        self.initial_t = 0.0
        self.default_q1 = np.deg2rad(0.0)
        self.default_q2 = np.deg2rad(80.0)
        self.default_d3 = 0.03
        
        # Coloar o robô na posição de início
        self.robot_plot.update_robot_position(self.default_q1, self.default_q2, self.default_d3)    # self.initial_t, 
        self.update_trajectory_params()
        self.update_coordenate_system()

        # Extra: validação própria
        # print("q0:\t", np.round([np.rad2deg(self.q0[0]), np.rad2deg(self.q0[1]), 100*self.q0[2]], 2))
        # print("qf:\t", np.round([np.rad2deg(self.qf[0]), np.rad2deg(self.qf[1]), 100*self.qf[2]], 2))
        # print("pos0:\t", np.round(self.pos0, 2))
        # print("posf:\t", np.round(self.pos0, 2))

        # --- Servidor TCP (PC) <-> ESP32 ---
        self.srv = ESP32Server(self)

        self.srv.status.connect(self.update_status)
        self.srv.error_occurred.connect(self._on_comm_error)
        self.srv.text_received.connect(self._on_text_line)
        self.srv.json_received.connect(self._on_json_obj)

        self.srv.client_connected.connect(lambda ip, port: self.update_status(f"ESP32 conectada: {ip}:{port}"))
        self.srv.client_disconnected.connect(lambda: self.update_status("ESP32 desconectada"))
        self.srv.server_started.connect(lambda p: self.update_status(f"Servidor iniciado na porta {p}"))
        self.srv.server_stopped.connect(lambda: self.update_status("Servidor parado"))

        # (opcional) estado para guardar último ref/meas
        self.last_ref = None
        self.last_meas = None
        self.last_meas_t = None

        # Pegando os dados do slider só depois do gripper ficar pronto
        # self.gripper_slider.valueChanged.connect(self.on_gripper_slider_changed)
        # enquanto isso recebo dado de outro lugar
        self.initial_q1.textChanged.connect(self.on_gripper_slider_changed)

    """--------------------------- Inicialização das outras Classes ---------------------------"""
    def init_components(self):
        """Inicializar componentes"""
        try:
            self.robot = CocoaBot()
            self.trajectory_gen = TrajectoryGenerator()
            self.simulation_thread = SimulationThread()
            self.trajectory = []
            self.simulation_thread.tx_json.connect(self.send_esp32_json)

            
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
        layout.addWidget(self.create_control_group())
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

        # Campos de entrada: Sistema de Coordenadas
        self.radio_joint = QRadioButton("Juntas")
        self.radio_cartesian = QRadioButton("Cartesiano")
        self.radio_joint.setChecked(True)

        # Campos de entrada: q_0 e q_f | pos_0 e pos_f
        self.initial_q1 = QLineEdit("0")                    # [entrada] q_0[1]
        self.initial_q2 = QLineEdit("80")                   # [entrada] q_0[2]
        self.initial_d3 = QLineEdit("3.0")                  # [entrada] q_0[3]
        self.final_q1 = QLineEdit("90")                     # [entrada] q_f[1]
        self.final_q2 = QLineEdit("135")                    # [entrada] q_f[2]
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
        # Sistema de Coordenadas
        sistcoord_label = QLabel("Coordenadas:")            # (Linha 0): "Coordenadas" | "Juntas" | "Cartesiano"
        layout.addWidget(sistcoord_label, 0, 0)
        layout.addWidget(self.radio_joint, 0, 1)
        layout.addWidget(self.radio_cartesian, 0, 2)

        # Headers: 
        init_label = QLabel("Inicial")                      # (Linha 1): " " | "Inicial" | "Final"
        final_label = QLabel("Final")
        layout.addWidget(init_label, 1, 1, Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(final_label, 1, 2, Qt.AlignmentFlag.AlignCenter)
        
        # Linhas das juntas: 
        self.q1_label = QLabel("q1 [deg]:")                 # (Linha 2): "q1 [deg]:" | [entrada: initial_q1] | [entrada: final_q1]
        layout.addWidget(self.q1_label, 2, 0, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(self.initial_q1, 2, 1)
        layout.addWidget(self.final_q1, 2, 2)

        self.q2_label = QLabel("q2 [deg]:")                 # (Linha 3): "q2 [deg]:" | [entrada: initial_q2] | [entrada: final_q2]
        layout.addWidget(self.q2_label, 3, 0, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(self.initial_q2, 3, 1)
        layout.addWidget(self.final_q2, 3, 2)

        self.d3_label = QLabel("d3 [cm]:")                  # (Linha 4): "d3 [cm]:" | [entrada: initial_q3] | [entrada: final_q3]
        layout.addWidget(self.d3_label, 4, 0, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(self.initial_d3, 4, 1)
        layout.addWidget(self.final_d3, 4, 2)
        
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
        self.duration_field = QLineEdit("5")            # Duração
        self.dt_field = QLineEdit("50")                 # Dt
        # Validador
        self.duration_field.setValidator(double_validator)
        self.dt_field.setValidator(double_validator)
        # Altura
        self.duration_field.setMaximumHeight(35)
        self.dt_field.setMaximumHeight(35)
                
        # Adicionar os Headers
        dur_label = QLabel("Duração [s]")               # (Linha 0): "Duração [s] | [entrada: duration]"
        dt_label = QLabel("dt [ms]")                    # (Linha 1): "dt [ms] | [entrada: dt]"
        layout.addWidget(dur_label, 0, 0, Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(dt_label, 0, 1, Qt.AlignmentFlag.AlignCenter)

        # Adiciona os Campos
        layout.addWidget(self.duration_field, 1, 0)
        layout.addWidget(self.dt_field, 1, 1)

        # Adiciona os Botões
        button_style = """
            QPushButton {
                padding: 4px 8px;
                font-size: 11px;
                min-height: 20px;
                max-height: 25px;
            }
        """

        # Botão: calcular trajetória
        self.btn_calc_trajectory = QPushButton("Calcular Trajetória")
        self.btn_calc_trajectory.setStyleSheet(button_style)
        layout.addWidget(self.btn_calc_trajectory, 2, 0, 1, 2)
        
        # Botão: Enviar para o robô
        # self.btn_send_to_robot = QPushButton("Enviar para o Robô")
        # self.btn_send_to_robot.setStyleSheet(button_style)
        # layout.addWidget(self.btn_send_to_robot, 3, 0, 1, 2)

        return group

    """--------------------------- 1.2) Painel de Controle -> Campo de Trajetória ---------------------------"""
    def create_control_group(self):
        # Layout Geral
        group = QGroupBox("Controle")
        layout = QGridLayout(group)
        layout.setSpacing(4)
        
        # Checkbox Controlador                                  # (Linha 0): [checkbox] | "Usar controlador Torque Calculado"
        self.use_controller = QCheckBox("Usar controlador Torque Calculado")
        layout.addWidget(self.use_controller, 0, 0, 1, 3)
        self.use_controller.setChecked(True)

        # Validador
        double_validator = QDoubleValidator()
        double_validator.setDecimals(5)
        double_validator.setLocale(QLocale(QLocale.Language.English, QLocale.Country.UnitedStates))

        # Headers: ganhos Kp, Kd, Ki
        self.Kp_label = QLabel("Kp")                          # (Linha 1): " " | "Inicial" | "Final"
        self.Kd_label = QLabel("Kd")
        self.Ki_label = QLabel("Ki")
        layout.addWidget(self.Kp_label, 1, 0, Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.Kd_label, 1, 1, Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.Ki_label, 1, 2, Qt.AlignmentFlag.AlignCenter)

        # Campos para os ganhos
        self.Kp_field = QLineEdit("0.050")
        self.Kd_field = QLineEdit("0.150")
        self.Ki_field = QLineEdit("0.001")

        # Adiciona validador
        self.Kp_field.setValidator(double_validator)
        self.Kd_field.setValidator(double_validator)
        self.Ki_field.setValidator(double_validator)

        # Altura (Height)
        self.Kp_field.setMaximumHeight(35)
        self.Kd_field.setMaximumHeight(35)
        self.Ki_field.setMaximumHeight(35)

        # Adicionar os layout
        layout.addWidget(self.Kp_field, 2, 0)
        layout.addWidget(self.Kd_field, 2, 1)
        layout.addWidget(self.Ki_field, 2, 2)
        
        self.update_controller(True)
        return group

    """--------------------------- 1.3) Painel de Controle -> Campo de Simulação ---------------------------"""
    def create_simulation_group(self):
        """Controles de simulação"""
        # Layout Geral
        group = QGroupBox("Simulação")
        layout = QVBoxLayout(group)
        layout.setSpacing(4)
        
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
        self.speed_label = QLabel("1.0x")       # --> Velocidade inicial (simulação): 1.0x
        self.speed_label.setFont(QFont("Arial", 11))
        self.speed_label.setMinimumWidth(40)
        speed_layout.addWidget(self.speed_label)
        layout.addLayout(speed_layout)

        # Barra de progresso                                    # (Linha 2): [progress bar] (visível apenas quando começa a simulação)
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)
        
        # Botões                                                # (Linhas 3-6): Botões ("trajetória", "simular", "parar", "robô")
        self.btn_simulate = QPushButton("Simular")
        self.btn_stop = QPushButton("Pausar")
        # Adotar o estilo do botão
        button_style = """
            QPushButton {
                padding: 4px 8px;
                font-size: 11px;
                min-height: 20px;
                max-height: 25px;
            }
        """
        self.btn_simulate.setStyleSheet(button_style)
        self.btn_stop.setStyleSheet(button_style)
        # Alguns botões são desabilitados inicialmente
        self.btn_stop.setEnabled(False)
        self.btn_simulate.setEnabled(False)
        # Adiciona os Widgets
        layout.addWidget(self.btn_simulate)
        layout.addWidget(self.btn_stop)
        
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
        
        # Abas/Tabs
        self.tabs = QTabWidget()
        
        # Plots                                             # (Linha 1): [Plots: RobotPlot()]
        try:
            self.robot_plot = RobotPlot(robot=self.robot)
            # Adicionar cada canvas em uma aba diferente
            self.tabs.addTab(self.robot_plot.canvas_positioning, "Posicionamento")
            self.tabs.addTab(self.robot_plot.canvas_time_evolution, "Evolução Temporal")
            self.tabs.addTab(self.robot_plot.canvas_errors, "Erros")
            self.tabs.addTab(self.robot_plot.canvas_forces, "Forças|Torques")
        except Exception as e:
            error_label = QLabel(f"Erro: {str(e)}")
            error_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(error_label)
        
        # Adicionar às Abas
        layout.addWidget(self.tabs)
        
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
            self.btn_simulate.clicked.connect(self.start_stop_simulation)
            self.btn_stop.clicked.connect(self.pause_resume_simulation)
            self.speed_slider.valueChanged.connect(self.update_speed)
            self.use_controller.clicked.connect(self.update_controller)
            self.radio_joint.toggled.connect(self.update_position_labels)
            # self.btn_send_to_robot.clicked.connect(self.send_to_robot)

            # Entradas dados: Atualização dos dados (tempo real): textChanged
            self.initial_q1.textChanged.connect(self.update_coordenate_system)
            self.initial_q2.textChanged.connect(self.update_coordenate_system)
            self.initial_d3.textChanged.connect(self.update_coordenate_system)
            self.final_q1.textChanged.connect(self.update_coordenate_system)
            self.final_q2.textChanged.connect(self.update_coordenate_system)
            self.final_d3.textChanged.connect(self.update_coordenate_system)
            self.duration_field.textChanged.connect(self.update_trajectory_params)
            self.dt_field.textChanged.connect(self.update_trajectory_params)
            self.Kp_field.textChanged.connect(self.update_gains)
            self.Kd_field.textChanged.connect(self.update_gains)
            self.Ki_field.textChanged.connect(self.update_gains)
            
            # Conecta com a Thread de simulation_thread.py
            # Quando os sinais chegam, as funções de update são acionadas
            self.simulation_thread.position_updated.connect(self.update_robot_position)
            self.simulation_thread.status_updated.connect(self.update_status)
            self.simulation_thread.progress_updated.connect(self.update_progress)
            
        except Exception as e:
            self.update_status(f"Erro ao configurar conexões: {str(e)}")
    
    """
    =================================================================================================================
                                                0) Funções Auxiliares
    =================================================================================================================
    """

    """--------------------------- Leitura de dados: Juntas ---------------------------"""
    def read_joints(self):
        """Ler as entradas correspondentes às coordenadas de juntas"""
        q0 = np.array([
            np.deg2rad(float(self.initial_q1.text())),
            np.deg2rad(float(self.initial_q2.text())),
            float(self.initial_d3.text())/100.0
        ])
        qf = np.array([
            np.deg2rad(float(self.final_q1.text())),
            np.deg2rad(float(self.final_q2.text())),
            float(self.final_d3.text())/100.0
        ])
        return q0, qf
    
    """--------------------------- Leitura de dados: Cartesiano ---------------------------"""
    def read_cartesian(self):
        """Ler as entradas correspondentes às coordenadas cartesianas"""
        pos0 = np.array([
            float(self.initial_q1.text()),
            float(self.initial_q2.text()),
            float(self.initial_d3.text())
        ])
        posf = np.array([
            float(self.final_q1.text()),
            float(self.final_q2.text()),
            float(self.final_d3.text())
        ])
        return pos0, posf

    """--------------------------- Validação inicial ---------------------------"""
    def initial_validation(self, block=False):
        """Validação entradas nulas e não numéricas"""
        # Flag
        all_valid = True

        # Campos
        fields = [
            (self.initial_q1, "q1 inicial"), (self.initial_q2, "q2 inicial"), (self.initial_d3, "d3 inicial"),
            (self.final_q1, "q1 final"), (self.final_q2, "q2 final"), (self.final_d3, "d3 final"),
            (self.duration_field, "duração"), (self.dt_field, "dt"), 
            (self.Kp_field, "Kp"), (self.Kd_field, "Kd"), (self.Ki_field, "Ki")
        ]
        
        for field, name in fields:
            if not field.text().strip():                                                # Validação: entradas nulas
                all_valid = False
                field.setStyleSheet(self.error_style)
                if block:
                    QMessageBox.warning(self, "Campo Vazio", f"Preencha '{name}'")
                    return False
                continue

            try:                                                                        # Validação: não numéricos
                float(field.text())
            except ValueError:
                all_valid = False
                field.setStyleSheet(self.error_style)
                if block: 
                    QMessageBox.warning(self, "Valor Inválido", f"'{name}' deve ser numérico")
                    return False
                continue
            
            field.setStyleSheet(self.normal_style)

        return all_valid

    """--------------------------- Validação completa das entradas ---------------------------"""
    def validate_inputs(self, block=False):
        """Validação completa"""
        try:
            if not self.initial_validation(block):
                return False

            if not self.validate_trajectory_inputs(block):
                return False

            if self.radio_joint.isChecked():
                if not self.validate_joint_inputs(block):
                    return False
            
            elif self.radio_cartesian.isChecked():
                if not self.validate_cartesian_inputs(block):
                    return False
            
            return True
            
        except Exception as e:
            QMessageBox.critical(self, "Erro na Validação", f"Erro na Validação: {str(e)}")
            return False    

    """--------------------------- Validação da trajetória ---------------------------"""
    def validate_trajectory_inputs(self, block=False):
        """Validação das entradas do campo de trajetórias"""
        # Flag
        all_valid = True

        # --- [Validação]: Duração [(Duration > 0)], Dt [(0 < dt < duração)] ---
        # Validar dt
        if self.dt <= 0.0 or self.dt >= self.tf:
            all_valid = False
            self.dt_field.setStyleSheet(self.error_style)
            if block: 
                QMessageBox.warning(self, "Valor Inválido", "0 < dt < duração")
                self.dt_field.setFocus()
                return False    
        else:
            self.dt_field.setStyleSheet(self.normal_style)
        
        # Validar duração
        if self.tf <= 0.0:
            all_valid = False
            self.duration_field.setStyleSheet(self.error_style)
            if block: 
                QMessageBox.warning(self, "Valor Inválido", "Duração > 0")
                self.duration_field.setFocus()
                return False
        else:
            self.duration_field.setStyleSheet(self.normal_style)
        
        return all_valid
    
    """--------------------------- Validação dos limites operacionais (juntas) ---------------------------"""
    def validate_joint_inputs(self, block=False):
        """Validação dos limites operacionais em coordenadas de juntas"""
        # Flag
        all_valid = True

        # Dados
        q1_initial, q2_initial, d3_initial = self.q0
        q1_final, q2_final, d3_final = self.qf

        # Limites físicos
        d3_max = self.robot.d3_max
        q1_min, q1_max = self.robot.J1_range
        q2_min, q2_max = self.robot.J2_range

        # --- [Validação]: Qi [(RANGE_MIN <= q <= RANGE_MAX)] & D3 [(0 < D3 < DR_MAX)] ---
        # Validar q1 inicial
        if q1_initial < q1_min or q1_initial > q1_max:
            all_valid = False
            self.initial_q1.setStyleSheet(self.error_style)
            if block: 
                QMessageBox.warning(self, "Limite Físico", f"q1 inicial: {np.rad2deg(q1_min)} a {np.rad2deg(q1_max)} graus")
                self.initial_q1.setFocus()
                return False
        else:
            self.initial_q1.setStyleSheet(self.normal_style)
        
        # Validar q1 final
        if q1_final < q1_min or q1_final > q1_max:
            all_valid = False
            self.final_q1.setStyleSheet(self.error_style)
            if block: 
                QMessageBox.warning(self, "Limite Físico", f"q1 final: {np.rad2deg(q1_min)} a {np.rad2deg(q1_max)} graus")
                self.final_q1.setFocus()
                return False
        else:
            self.final_q1.setStyleSheet(self.normal_style)
        
        # Validar q2 inicial
        if q2_initial < q2_min or q2_initial > q2_max:
            all_valid = False
            self.initial_q2.setStyleSheet(self.error_style)
            if block: 
                QMessageBox.warning(self, "Limite Físico", f"q2 inicial: {np.rad2deg(q2_min)} a {np.rad2deg(q2_max)} graus")
                self.initial_q2.setFocus()
                return False
        else:
            self.initial_q2.setStyleSheet(self.normal_style)
        
        # Validar q2 final
        if q2_final < q2_min or q2_final > q2_max:
            all_valid = False
            self.final_q2.setStyleSheet(self.error_style)
            if block: 
                QMessageBox.warning(self, "Limite Físico", f"q2 final: {np.rad2deg(q2_min)} a {np.rad2deg(q2_max)} graus")
                self.final_q2.setFocus()
                return False
        else:
            self.final_q2.setStyleSheet(self.normal_style)
        
        # Validar d3 inicial
        if d3_initial < 0 or d3_initial > d3_max:
            all_valid = False
            self.initial_d3.setStyleSheet(self.error_style)
            if block: 
                QMessageBox.warning(self, "Limite Físico", f"d3 inicial: 0.00 a {100*d3_max:.2f} cm")
                self.initial_d3.setFocus()
                return False
        else:
            self.initial_d3.setStyleSheet(self.normal_style)
        
        # Validar d3 final
        if d3_final < 0 or d3_final > d3_max:
            all_valid = False
            self.final_d3.setStyleSheet(self.error_style)
            if block: 
                QMessageBox.warning(self, "Limite Físico", f"d3 final: 0.00 a {100*d3_max:.2f} cm")
                self.final_d3.setFocus()
                return False
        else:
            self.final_d3.setStyleSheet(self.normal_style)

        return all_valid

    """--------------------------- Validação dos limites operacionais (cartesiano) ---------------------------"""
    def validate_cartesian_inputs(self, block=False):
        """Validação dos limites operacionais em coordenadas cartesianas"""
        # Flag
        all_valid = True

        # Dados
        x0, y0, z0 = self.pos0
        xf, yf, zf = self.posf

        # Raio e Hiputenusa
        r0 = np.sqrt(x0**2 + y0**2)
        rf = np.sqrt(xf**2 + yf**2) 
        h0 = np.sqrt(x0**2 + y0**2 + (z0-self.robot.L1)**2)
        hf = np.sqrt(xf**2 + yf**2 + (zf-self.robot.L1)**2)

        # Limites físicos
        r_min, r_max = self.robot.r_min, self.robot.r_max
        h_min, h_max = self.robot.h_min, self.robot.h_max

        # --- [Validação]: R [(R_MIN <= R <= R_MAX)] ---
        # Validar R inicial
        if r0 < r_min or r0 > r_max:
            all_valid = False
            self.initial_q1.setStyleSheet(self.error_style)
            self.initial_q2.setStyleSheet(self.error_style)
            if block: 
                QMessageBox.warning(self, "Limite Físico", f"Raio inicial (XY): {r_min:.2f} a {r_max:.2f} m")
                return False
        else:
            self.initial_q1.setStyleSheet(self.normal_style)
            self.initial_q2.setStyleSheet(self.normal_style)

        # Validar R final
        if rf < r_min or rf > r_max:
            all_valid = False
            self.final_q1.setStyleSheet(self.error_style)
            self.final_q2.setStyleSheet(self.error_style)
            if block: 
                QMessageBox.warning(self, "Limite Físico", f"Raio final (XY): {r_min:.2f} a {r_max:.2f} m")
                return False
        else:
            self.final_q1.setStyleSheet(self.normal_style)
            self.final_q2.setStyleSheet(self.normal_style)
        
        # Se R estiver inválido, não verifica Z
        if not all_valid:
            self.initial_d3.setStyleSheet(self.normal_style)
            self.final_d3.setStyleSheet(self.normal_style)
            return False

        # --- [Validação]: Z [(H_MIN <= H <= H_MAX)] & Z [(Z_MIN(i) < Z < Z_MAX(i))] ---
        # Limites físicos (não mudar de posição no código)
        z0_vec = np.sort(self.robot.calc_Zrange(r0))
        zf_vec = np.sort(self.robot.calc_Zrange(rf))

        # Validar Z inicial
        if z0_vec is None:
            all_valid = False
        else:
            if h0 < h_min or h0 > h_max:
                all_valid = False
                self.initial_d3.setStyleSheet(self.error_style)
                if block: 
                    QMessageBox.warning(self, "Limite Físico", f"Z inicial (RZ) fora do range")
                    return False
            else:
                idx_sep_z0 = np.argmax(np.diff(z0_vec)) + 1
                z0_lesser = z0_vec[:idx_sep_z0]
                z0_greater = z0_vec[idx_sep_z0:]
                if z0 < np.min(z0_lesser) or (z0 > np.max(z0_lesser) and z0 < np.min(z0_greater)) or z0 > np.max(z0_greater):
                    all_valid = False
                    self.initial_d3.setStyleSheet(self.error_style)
                    if block: 
                        QMessageBox.warning(self, "Limite Físico", f"Z inicial (RZ) fora do range")
                        return False
                else:
                    self.initial_d3.setStyleSheet(self.normal_style)
            
        # Validar Z final
        if zf_vec is None:
            all_valid = False
        else:
            if hf < h_min or h0 > h_max:
                all_valid = False
                self.final_d3.setStyleSheet(self.error_style)
                if block: 
                    QMessageBox.warning(self, "Limite Físico", f"Z inicial (RZ) fora do range")
                    return False
            else:
                idx_sep_zf = np.argmax(np.diff(zf_vec)) + 1
                zf_lesser = zf_vec[:idx_sep_zf]
                zf_greater = zf_vec[idx_sep_zf:]
                if zf < np.min(zf_lesser) or (zf > np.max(zf_lesser) and zf < np.min(zf_greater)) or zf > np.max(zf_greater):
                    all_valid = False
                    self.final_d3.setStyleSheet(self.error_style)
                    if block: 
                        QMessageBox.warning(self, "Limite Físico", f"Z inicial (RZ) fora do range")
                        return False
                else:
                    self.final_d3.setStyleSheet(self.normal_style)
        
        return all_valid

    """--------------------------- (Des)Habilita Botões durante a Simulação ---------------------------"""
    def enable_simulation_buttons(self, enable):
            """(Des)Habilita Botões durante a Simulação"""
            has_trajectory = True if hasattr(self, 'trajectory') and self.trajectory else False
            if not enable: self.btn_simulate.setEnabled(has_trajectory)
            self.btn_stop.setEnabled(enable)
            self.btn_calc_trajectory.setEnabled(not enable)
            self.progress_bar.setVisible(enable)
            if enable: self.progress_bar.setValue(0)
            self.radio_joint.setEnabled(not enable)
            self.radio_cartesian.setEnabled(not enable)
            self.enable_position_buttons(not enable)
            self.duration_field.setEnabled(not enable)
            self.dt_field.setEnabled(not enable)
            self.use_controller.setEnabled(not enable)
            self.Kp_field.setEnabled(not enable)
            self.Kd_field.setEnabled(not enable)
            self.Ki_field.setEnabled(not enable)

    """--------------------------- (Des)Habilita Botões de Posição ---------------------------"""
    def enable_position_buttons(self, enable):
        """(Des)Habilita Botões de Posição"""
        self.initial_q1.setEnabled(enable)
        self.initial_q2.setEnabled(enable)
        self.initial_d3.setEnabled(enable)
        self.final_q1.setEnabled(enable)
        self.final_q2.setEnabled(enable)
        self.final_d3.setEnabled(enable)
    
    """--------------------------- Limpar Interface ---------------------------"""
    def clear_interface(self):
        """Limpa a interface"""
        try:
            if hasattr(self, 'robot_plot'):
                self.robot_plot.update_robot_position(*self.q0)     # Voltar o robô pra posição inicial
                self.robot_plot.reset_position_graph()
                self.robot_plot.reset_trajectory()
                self.robot_plot.reset_positioning_lines()
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao limpar a interface: {str(e)}")
    
    """
    =================================================================================================================
                                                1) Funções de Controle
    =================================================================================================================
    """

    """--------------------------- 1.1) Funções de Controle -> Calcula Trajetória ---------------------------"""
    def calculate_trajectory(self):
        """Calcular trajetória"""
        try:
            if not self.validate_inputs(block=True):
                return
            
            # Gera a Trajetória: trajectory = [(t, q_t, qd_t, qdd_t), (...)]
            self.trajectory = self.trajectory_gen.generate_trajectory(self.q0, self.qf, self.tf, self.dt)
            
            # [Update plot]: posiciona o robô, adiciona a Trajetória Completa e a Posição Final nos plots
            if hasattr(self, 'robot_plot'):
                self.clear_interface()
                self.robot_plot.set_trajectory(self.trajectory)                     # Adiciona a Trajetória Completa
                self.robot_plot.set_target_position(*self.posf)                     # Adiciona a Posição Final
            
            # Ativa o Botão de "Simular"
            self.btn_simulate.setEnabled(True)

            # Adiciona status no log
            self.update_status(f"Trajetória: {len(self.trajectory)} pts, {self.tf}s")
            
        except ValueError as e:
            QMessageBox.warning(self, "Entrada Inválida", f"Verifique os valores: {str(e)}")
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao calcular trajetória: {str(e)}")
    
    """--------------------------- 1.2) Funções de Controle -> Começar/Cancelar Simulação ---------------------------"""
    def start_stop_simulation(self):
        if self.btn_simulate.text() == "Simular":
            if not hasattr(self, 'trajectory') or not self.trajectory:
                QMessageBox.warning(self, "Aviso", "Calcule trajetória primeiro!")
                return
            self.start_simulation()
            self.btn_simulate.setText("Cancelar")
        elif self.btn_simulate.text() == "Cancelar":
            self.stop_simulation()
            self.btn_simulate.setText("Simular")
            self.btn_stop.setText("Pausar")

    """--------------------------- 1.3) Funções de Controle -> Começar Simulação ---------------------------"""
    def start_simulation(self):
        """Iniciar simulação"""
        try:        
            if not self.validate_inputs(block=True):
                return

            if hasattr(self, 'robot_plot'):
                self.robot_plot.reset_position_graph()
                self.robot_plot.reset_positioning_lines()
            
            # Passa a trajetória e o controle para o thread                 # Configuração da Simulation_Thread
            self.simulation_thread.set_trajectory(self.trajectory)
            
            # Ativa/desativa os botões
            self.enable_simulation_buttons(True)
            
            # Inicia a simulação na thread                                  # Início da Simulation_Thread
            self.simulation_thread.start()
            
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao iniciar simulação: {str(e)}")
            self.enable_simulation_buttons(False)
    
    """--------------------------- 1.4) Funções de Controle -> Cancelar Simulação ---------------------------"""
    def stop_simulation(self):
        """Parar simulação"""
        try:
            self.simulation_thread.stop()                                   # Cancelamento da Simulation_Thread
            QTimer.singleShot(1000, lambda: self.update_status("Simulação parada"))
            self.update_coordenate_system()
            self.enable_simulation_buttons(False)
            self.trajectory = None

        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao parar simulação: {str(e)}")
    
    """--------------------------- 1.5) Funções de Controle -> Pausar/Retomar Simulação ---------------------------"""
    def pause_resume_simulation(self):
        action = self.btn_stop.text()

        try:
            if action == "Pausar":                                          # Pausar simulação
                self.simulation_thread.pause()
                QTimer.singleShot(50, lambda: self.update_status("Simulação pausada"))
                self.btn_stop.setText("Retomar")
                
            elif action == "Retomar":                                       # Retomar simulação
                self.update_status("Simulação retomada")
                self.simulation_thread.resume()
                self.btn_stop.setText("Pausar")

        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao {action.lower()} simulação: {str(e)}")
    
    """--------------------------- 1.6) Funções de Controle -> Enviar para o Robô ---------------------------"""
    def send_to_robot(self):
        pass

    """
    =================================================================================================================
                                                2) Funções de Update (Tempo real)
    =================================================================================================================
    """

    """--------------------------- 2.1) Funções de Update -> Velocidade de simulação ---------------------------"""
    def update_speed(self, value):
        """Atualiza velocidade: altera o tempo de sleep() de atualização dos plots"""
        try:
            speed = value / 10.0
            self.speed_label.setText(f"{speed:.1f}x")
            self.simulation_thread.set_speed(speed)
        except Exception as e:
            print(f"Erro ao atualizar velocidade: {e}")
    
    """--------------------------- 2.2) Funções de Update -> Controlador ---------------------------"""
    def update_controller(self, checked):
        """Verifica se o controlador será utilizado"""
        try:
            if checked:
                self.controller = CalculatedTorqueController(self.robot)
                self.update_gains()
            else:
                if hasattr(self, 'controller'):
                    self.controller = None

            self.simulation_thread.set_controller(self.controller)
            
            # Alterar a visibilidade dos Campos
            self.Kp_label.setVisible(checked)
            self.Kd_label.setVisible(checked)
            self.Ki_label.setVisible(checked)
            self.Kp_field.setVisible(checked)
            self.Kd_field.setVisible(checked)
            self.Ki_field.setVisible(checked)
        
        except Exception as e:
            print(f"Erro ao atualizar controlador: {e}")
    
    """--------------------------- 2.3) Funções de Update -> Posição e Trajetória das juntas ---------------------------"""
    def update_robot_position(self, t, q, qd, qdd, e, ed, edd, tau):
        """Atualizar gráficos de posição"""                                     # [sinal]: <- simulation_thread.position_updated
        try:
            if hasattr(self, 'robot_plot'):
                self.robot_plot.update_robot_position(*q)                               # [update]: -> robot_plot.update_robot_position() [plot: 3d, xy, rz]
                self.robot_plot.update_time_evolution(t, q, qd, qdd, e, ed, edd, tau)   # [update]: -> robot_plot.update_time_evolution() [plot: posicao da junta x tempo]
        except Exception as e:
            print(f"Erro ao atualizar posição: {e}")
    
    """--------------------------- 2.4) Funções de Update -> Status (log) ---------------------------"""
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
    
    """--------------------------- 2.5) Funções de Update -> Barra de Progresso ---------------------------"""
    def update_progress(self, value):
        """Atualizar progresso"""                                               # [sinal]: <- simulation_thread.progress_updated
        try:
            self.progress_bar.setValue(value)                                   # [update]: -> progress_bar.setValue(value) [atualiza a barra de progresso]
            if value >= 100:
                self.btn_simulate.setText("Simular")
                QTimer.singleShot(1000, lambda: self.enable_simulation_buttons(False))
        except Exception as e:
            print(f"Erro ao atualizar progresso: {e}")
    
        """--------------------------- Em construção (!!!!) ---------------------------"""
    
    """--------------------------- 2.6) Funções de Update -> Parâmetros da Trajetória ---------------------------"""
    def update_trajectory_params(self):
        """Atualizar as variáveis do campo de trajetória"""
        try:
            if not self.initial_validation(block=False):
                return
            
            # Duração e dt
            self.tf = float(self.duration_field.text())
            self.dt = float(self.dt_field.text()) / 1000

            # Validação
            self.validate_trajectory_inputs(block=False)

        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro na configuração da trajetória: {str(e)}")
    
    """--------------------------- 2.7) Funções de Update -> Mudança do Sistema de Coordenadas ---------------------------"""
    def update_position_labels(self):
        """Mudança de sistema de coordenadas: campos da ui e valores internos"""
        with QSignalBlocker(self.initial_q1), QSignalBlocker(self.initial_q2), \
            QSignalBlocker(self.initial_d3), QSignalBlocker(self.final_q1), \
            QSignalBlocker(self.final_q2), QSignalBlocker(self.final_d3):

            if self.radio_joint.isChecked():
                # print("Cartesian -> Joint")
                self.update_status("Transformação: cartesiano -> juntas")
                self.q1_label.setText("q1 [deg]:")
                self.q2_label.setText("q2 [deg]:")
                self.d3_label.setText("d3 [cm]:")
            elif self.radio_cartesian.isChecked():
                # print("Joint -> Cartesian")
                self.update_status("Transformação: juntas -> cartesiano")
                self.q1_label.setText("x [m]:")
                self.q2_label.setText("y [m]:")
                self.d3_label.setText("z [m]:")
            self.set_transformed_inputs()
        
        self.update_coordenate_system()

        if not self.validate_inputs(block=False):
            self.update_status("Aviso: Configuração fora dos limites operacionais")

        # Extra: validação própria
        # print("q0:\t", np.round([np.rad2deg(self.q0[0]), np.rad2deg(self.q0[1]), 100*self.q0[2]], 2))
        # print("qf:\t", np.round([np.rad2deg(self.qf[0]), np.rad2deg(self.qf[1]), 100*self.qf[2]], 2))
        # print("pos0:\t", np.round(self.pos0, 2))
        # print("posf:\t", np.round(self.pos0, 2))
    
    """--------------------------- 2.7.1) Funções de Update -> Setar dados transformados no Campo de Posição ---------------------------"""
    def set_transformed_inputs(self):
        try:
            if self.radio_joint.isChecked():
                data0 = [np.rad2deg(self.q0[0]), np.rad2deg(self.q0[1]), 100*self.q0[2]]    # (q1,q2,d3) -> [deg, deg, cm]
                dataf = [np.rad2deg(self.qf[0]), np.rad2deg(self.qf[1]), 100*self.qf[2]]
            elif self.radio_cartesian.isChecked():
                data0 = self.pos0
                dataf = self.posf
            
            # Formatação
            data0 = np.round(data0, 2)
            dataf = np.round(dataf, 2)
            
            # Alocação dos dados
            self.initial_q1.setText(str((data0[0])))
            self.initial_q2.setText(str(data0[1]))
            self.initial_d3.setText(str(data0[2]))
            self.final_q1.setText(str(dataf[0]))
            self.final_q2.setText(str(dataf[1]))
            self.final_d3.setText(str(dataf[2]))

        except Exception as e:
            self.update_status(f"Erro ao alterar o sistema de coordenadas: {str(e)}")
    
    """--------------------------- 2.7.2) Funções de Update -> Transformação de Coordenadas: juntas <-> cartesiano ---------------------------"""
    def update_coordenate_system(self):
        """Cálculo da transformação de coordenadas: forward- or inverse_kinematics"""
        try:
            if not self.initial_validation(block=False):
                return

            # Cartesian -> joints
            if self.radio_joint.isChecked():
                self.q0, self.qf = self.read_joints()
                self.pos0 = self.robot.forward_kinematics(*self.q0)
                self.posf = self.robot.forward_kinematics(*self.qf)

            # Joints -> cartesian
            elif self.radio_cartesian.isChecked():
                self.pos0, self.posf = self.read_cartesian()
                self.q0 = self.robot.inverse_kinematics(*self.pos0)
                self.qf = self.robot.inverse_kinematics(*self.posf)

            # Validação
            self.validate_inputs(block=False)

            # Plots
            self.clear_interface()
            if hasattr(self, 'robot_plot'):
                self.robot_plot.set_target_position(*self.posf)
                if self.radio_cartesian.isChecked():
                    self.robot_plot.set_cartesian_positionining_lines(self.pos0, self.posf)
                elif self.radio_joint.isChecked():
                    self.robot_plot.set_joint_positionining_lines(self.q0, self.qf)
        
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro na transformação do sistema de coordenada: {str(e)}")
    
    """--------------------------- 2.8) Funções de Update -> Ganhos do Controlador ---------------------------"""
    def update_gains(self):
        try:
            if not self.initial_validation(block=False):
                return
            
            if not hasattr(self, "controller") or self.controller:
                return
            
            Kp_scaling_factor = float(self.Kp_field.text())
            Kd_scaling_factor = float(self.Kd_field.text())
            Ki_scaling_factor = float(self.Ki_field.text())

            self.controller.set_gain_factors(Kp_scaling_factor, Kd_scaling_factor, Ki_scaling_factor)
            
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao atualizar os ganhos: {str(e)}")

    """"-------------------- Daddos do gripper ---------------"""

    # --- Código para usar com slider ---
    # def on_gripper_slider_changed(self, value: int):
    #     """
    #     Chamada SEMPRE que o slider da garra mudar.
    #     """
    #     
    #     # Isso garante que o 'run()' sempre terá o valor mais recente
    #     if hasattr(self, "simulation_thread"):
    #          self.simulation_thread.set_current_gripper_value(value)
    #
    #     # Lógica de Envio
    #     if self.simulation_thread and self.simulation_thread.is_running:
    #         # ...
    #         pass
    #     else:
    #         # Simulação PARADA: Envie o comando "direto"
    #         command = {
    #             "cmd": "set_gripper", # O comando separado
    #             "value": value
    #         }
    #         self.send_esp32_json(command)

    # --- C[odigo de teste ---
    def on_gripper_slider_changed(self): # <<< MODIFICADO: Não recebe 'value'
        """
        Chamada SEMPRE que o TEXTO em 'initial_q1' mudar.
        (Função original do slider comentada acima)
        """
        
        # 1. PEGA o valor da caixa de texto e CONVERTE (como você sugeriu)
        try:
            value = int(float(self.initial_q1.text())) # <<< MODIFICADO
        except ValueError:
            value = 0 # Se o texto for inválido (ex: "abc" ou vazio)

        # 2. A SUA LÓGICA ORIGINAL (copiada de cima, sem mudanças)
        if hasattr(self, "simulation_thread"):
             self.simulation_thread.set_current_gripper_value(value)

        # Lógica de Envio
        if self.simulation_thread and self.simulation_thread.is_running:
            pass
        else:
            command = {
                "cmd": "set_gripper",
                "value": value
            }
            self.send_esp32_json(command)
    
    """"-------------------- Comunicação ---------------"""

   # ===== Servidor no PC =====
    def start_esp32_server(self, port: int = 9000):
        self.srv.start(port)

    def stop_esp32_server(self):
        self.srv.stop()

    def send_esp32_text(self, text: str):
        self.srv.send_text(text)

    def send_esp32_json(self, payload: dict):
        self.srv.send_json(payload)

   
    def connect_esp32(self, host: str, port: int):
        # host é ignorado: o servidor escuta em 0.0.0.0:port
        self.start_esp32_server(port)

    def disconnect_esp32(self):
        self.stop_esp32_server()

    # ===== Handlers dos sinais da comunicação =====
    def _on_comm_error(self, msg: str):
        self.update_status(f"Erro ESP32: {msg}")

    def _on_text_line(self, line: str):
        self.update_status(f"RX texto: {line}")

    def _on_json_obj(self, obj: dict):
        if "pong" in obj:
            self.update_status("RX json: pong")
            return

        if "ref" in obj:
            self.last_ref = obj.get("ref")
            self.update_status(f"RX json (ref): {self.last_ref}")
            return

        if "meas_q" in obj:
            meas_q = obj.get("meas_q")
            meas_gripper = obj.get("meas_gripper")
            tval = obj.get("t")

            if meas_q is None or meas_gripper is None:
                self.update_status(f"Erro RX: Pacote 'meas_q' ou 'meas_gripper' ausentes.")
                return

            try:
                if isinstance(meas_q, list) and len(meas_q) >= 3:
                    all_zero = (float(meas_q[0]) == 0.0 and float(meas_q[1]) == 0.0 and float(meas_q[2]) == 0.0)
                    if all_zero:
                        self.update_status("RX meas: Pacote [0,0,0] ignorado")
                        return
            except Exception:
                pass

            self.last_meas = meas_q
            self.last_meas_gripper = meas_gripper
            self.last_meas_t = tval

            if hasattr(self, "simulation_thread") and self.simulation_thread is not None:
                try:
                    t_num = float(tval) if tval is not None else 0.0
                except Exception:
                    t_num = 0.0
                
                self.simulation_thread.ingest_meas_sig.emit(
                    self.last_meas,         # q
                    self.last_meas_gripper, # gripper
                    t_num                   # t
                )

            self.update_status(f"RX meas (q): {self.last_meas}")
            return

        # fallback
        self.update_status(f"RX json: {obj}")



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
