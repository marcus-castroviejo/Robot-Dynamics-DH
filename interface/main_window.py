"""
============================================
    Classe RobotControlInterface
============================================
- Interface principal

É aqui onde se cria as janelas com painés de controle e visualizaça7o
- Campos de entrada, botões, validação, ...

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
from dynamics import CocoaBot
from trajectory import TrajectoryGeneratorRRP
from visualization import RobotPlot3D
# from interface import SimulationThread  # Da errado: importação circular com __init__.py !!!
from .simulation_thread import SimulationThread


class RobotControlInterface(QMainWindow):
    """Interface principal corrigida"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Controle CocoaBot")
        self.setGeometry(100, 100, 1600, 1000)
        # self.showMaximized()
        # self.setup_window_geometry()
        # self.setup_responsive_window()
        
        # Inicializar componentes
        self.init_components()      # Inicializa o robô, a trajetória e a thread de simulação
        self.init_ui()
        self.setup_connections()
        
        # Posição inicial padrão
        self.current_q1 = 0.0
        self.current_q2 = np.pi/4
        self.current_d3 = 0.05
        
        # Atualizar visualização inicial
        self.update_robot_position(self.current_q1, self.current_q2, self.current_d3)
    
    def setup_window_geometry(self):
        """Configurar geometria da janela baseada na tela"""
        # Obter informações da tela principal
        screen = QApplication.primaryScreen().geometry()
        screen_width = screen.width()
        screen_height = screen.height()
        
        # Calcular tamanho da janela
        window_width = int(screen_width * 0.90)
        window_height = int(screen_height * 0.90)
        
        # Centralizar na tela
        x = (screen_width - window_width) // 2
        y = (screen_height - window_height) // 2
        
        # Aplicar geometria
        self.setGeometry(x, y, window_width, window_height)
        
        # Definir tamanho mínimo
        self.setMinimumSize(1200, 800)

    def setup_responsive_window(self):
        """Janela responsiva que se adapta à tela"""
        screen = QApplication.primaryScreen().geometry()
        
        # Tamanhos mínimos para funcionalidade completa
        min_width = 1400
        min_height = 900
        
        # Usar 85% da tela, mas pelo menos o mínimo
        width = max(min_width, int(screen.width() * 0.85))
        height = max(min_height, int(screen.height() * 0.85))
        
        # Centralizar
        x = (screen.width() - width) // 2
        y = (screen.height() - height) // 2
        
        self.setGeometry(x, y, width, height)
        self.setMinimumSize(min_width, min_height)

    def init_components(self):
        """Inicializar componentes"""
        try:
            self.robot = CocoaBot()
            self.trajectory_gen = TrajectoryGeneratorRRP()
            self.simulation_thread = SimulationThread()
            self.trajectory = []
            
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao inicializar componentes: {str(e)}")
    
    def init_ui(self):
        """Interface do usuário corrigida"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Layout principal com splitter
        main_layout = QHBoxLayout(central_widget)
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
    
    def create_control_panel(self):
        """Painel de controles corrigido"""
        panel = QFrame()
        panel.setFrameStyle(QFrame.Shape.StyledPanel)
        panel.setMinimumWidth(350)
        panel.setMaximumWidth(400)
        
        layout = QVBoxLayout(panel)
        
        # Título
        title = QLabel("Controle CocoaBot")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("QLabel { color: #2E7D32; padding: 10px; }")
        layout.addWidget(title)
        
        # Posição inicial
        layout.addWidget(self.create_initial_position_group())
        
        # Posição final
        layout.addWidget(self.create_final_position_group())
        
        # Parâmetros de trajetória
        layout.addWidget(self.create_trajectory_params_group())
        
        # Controles de simulação
        layout.addWidget(self.create_simulation_group())
        
        # Status
        layout.addWidget(self.create_status_group())
        
        layout.addStretch()
        return panel
    
    def create_initial_position_group(self):
        """Grupo de posição inicial"""
        group = QGroupBox("Posição Inicial (Juntas)")
        layout = QGridLayout(group)
        
        # Validadores
        double_validator = QDoubleValidator()
        double_validator.setDecimals(4)
        
        self.initial_q1 = QLineEdit("0.0")
        self.initial_q1.setValidator(double_validator)
        self.initial_q2 = QLineEdit("0.7854")  # π/4
        self.initial_q2.setValidator(double_validator)
        self.initial_d3 = QLineEdit("0.05")
        self.initial_d3.setValidator(double_validator)
        
        layout.addWidget(QLabel("q1 (rad):"), 0, 0)
        layout.addWidget(self.initial_q1, 0, 1)
        layout.addWidget(QLabel("q2 (rad):"), 1, 0)
        layout.addWidget(self.initial_q2, 1, 1)
        layout.addWidget(QLabel("d3 (m):"), 2, 0)
        layout.addWidget(self.initial_d3, 2, 1)
        
        return group
    
    def create_final_position_group(self):
        """Grupo de posição final"""
        group = QGroupBox("Posição Final (Juntas)")
        layout = QGridLayout(group)
        
        # Validadores
        double_validator = QDoubleValidator()
        double_validator.setDecimals(4)
        
        self.final_q1 = QLineEdit("1.5708")  # π/2
        self.final_q1.setValidator(double_validator)
        self.final_q2 = QLineEdit("2.3562")  # 3π/4
        self.final_q2.setValidator(double_validator)
        self.final_d3 = QLineEdit("0.08")
        self.final_d3.setValidator(double_validator)
        
        layout.addWidget(QLabel("q1 (rad):"), 0, 0)
        layout.addWidget(self.final_q1, 0, 1)
        layout.addWidget(QLabel("q2 (rad):"), 1, 0)
        layout.addWidget(self.final_q2, 1, 1)
        layout.addWidget(QLabel("d3 (m):"), 2, 0)
        layout.addWidget(self.final_d3, 2, 1)
        
        return group
    
    def create_trajectory_params_group(self):
        """Grupo de parâmetros de trajetória"""
        group = QGroupBox("Parâmetros de Trajetória")
        layout = QGridLayout(group)
        
        # Validadores
        double_validator = QDoubleValidator()
        double_validator.setDecimals(3)
        double_validator.setBottom(0.001)
        
        self.duration = QLineEdit("5.0")
        self.duration.setValidator(double_validator)
        self.dt = QLineEdit("0.01")
        self.dt.setValidator(double_validator)
        
        layout.addWidget(QLabel("Duração (s):"), 0, 0)
        layout.addWidget(self.duration, 0, 1)
        layout.addWidget(QLabel("dt (s):"), 1, 0)
        layout.addWidget(self.dt, 1, 1)
        
        return group
    
    def create_simulation_group(self):
        """Grupo de controles de simulação"""
        group = QGroupBox("Controle de Simulação")
        layout = QVBoxLayout(group)
        
        # Usar controlador
        self.use_controller = QCheckBox("Usar Controlador PID")
        layout.addWidget(self.use_controller)
        
        # Velocidade
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Velocidade:"))
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(1, 50)
        self.speed_slider.setValue(10)
        self.speed_label = QLabel("1.0x")
        self.speed_label.setMinimumWidth(40)
        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_label)
        layout.addLayout(speed_layout)
        
        # Barra de progresso
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)
        
        # Botões
        button_layout = QVBoxLayout()
        
        self.btn_calc_trajectory = QPushButton("Calcular Trajetória")
        self.btn_simulate = QPushButton("Simular")
        self.btn_stop = QPushButton("Parar")
        self.btn_send_robot = QPushButton("Enviar para Robô")
        
        # Inicialmente desabilitar alguns botões
        self.btn_simulate.setEnabled(False)
        self.btn_stop.setEnabled(False)
        
        button_layout.addWidget(self.btn_calc_trajectory)
        button_layout.addWidget(self.btn_simulate)
        button_layout.addWidget(self.btn_stop)
        button_layout.addWidget(self.btn_send_robot)
        
        layout.addLayout(button_layout)
        return group
    
    def create_status_group(self):
        """Grupo de status"""
        group = QGroupBox("Status")
        layout = QVBoxLayout(group)
        
        self.status_text = QTextEdit()
        self.status_text.setMaximumHeight(120)
        self.status_text.setReadOnly(True)
        self.status_text.setFont(QFont("Monaco, Consolas, Monospace", 9))
        # self.status_text.setFont(QFont("Courier", 9))
        
        layout.addWidget(self.status_text)
        return group
    
    def create_visualization_panel(self):
        """Painel de visualização"""
        panel = QFrame()
        panel.setFrameStyle(QFrame.Shape.StyledPanel)
        layout = QVBoxLayout(panel)
        
        # Título
        title = QLabel("Visualização 3D - Robô RRP")
        title.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)
        
        # Plot 3D do robô
        try:
            self.robot_plot = RobotPlot3D()
            layout.addWidget(self.robot_plot)
        except Exception as e:
            error_label = QLabel(f"Erro ao carregar visualização: {str(e)}")
            error_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(error_label)
        
        return panel
    
    def setup_connections(self):
        """Configurar conexões de sinais"""
        try:
            # Botões
            self.btn_calc_trajectory.clicked.connect(self.calculate_trajectory)
            self.btn_simulate.clicked.connect(self.start_simulation)
            self.btn_stop.clicked.connect(self.stop_simulation)
            self.btn_send_robot.clicked.connect(self.send_to_robot)
            
            # Slider
            self.speed_slider.valueChanged.connect(self.update_speed)
            
            # Thread de simulação
            self.simulation_thread.position_updated.connect(self.update_robot_position)
            self.simulation_thread.status_updated.connect(self.update_status)
            self.simulation_thread.progress_updated.connect(self.update_progress)
            
        except Exception as e:
            self.update_status(f"Erro ao configurar conexões: {str(e)}")
    
    def calculate_trajectory(self):
        """Calcular trajetória"""
        try:
            # Validar entradas
            if not self.validate_inputs():
                return
            
            # Posições inicial e final
            q0 = np.array([
                float(self.initial_q1.text()),
                float(self.initial_q2.text()),
                float(self.initial_d3.text())
            ])
            
            qf = np.array([
                float(self.final_q1.text()),
                float(self.final_q2.text()),
                float(self.final_d3.text())
            ])
            
            # Parâmetros
            tf = float(self.duration.text())
            dt = float(self.dt.text())
            
            # Gerar trajetória
            self.trajectory = self.trajectory_gen.generate_trajectory(q0, qf, tf, dt)
            
            # Calcular pontos da trajetória no espaço cartesiano
            trajectory_points = []
            for _, q_joints, _, _ in self.trajectory:
                pos = self.robot.forward_kinematics(*q_joints)
                trajectory_points.append(pos)
            
            # Atualizar visualização
            if hasattr(self, 'robot_plot'):
                self.robot_plot.update_trajectory(trajectory_points)
                
                # Posição final no espaço cartesiano
                x_final, y_final, z_final = self.robot.forward_kinematics(*qf)
                self.robot_plot.set_target_position(x_final, y_final, z_final)
            
            # Habilitar botão de simulação
            self.btn_simulate.setEnabled(True)
            
            self.update_status(f"Trajetória calculada: {len(self.trajectory)} pontos, duração: {tf}s")
            
        except ValueError as e:
            QMessageBox.warning(self, "Entrada Inválida", f"Verifique os valores inseridos: {str(e)}")
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao calcular trajetória: {str(e)}")
            self.update_status(f"Erro: {str(e)}")
    
    def validate_inputs(self):
        """Validar entradas do usuário"""
        try:
            # Verificar se todos os campos estão preenchidos
            fields = [
                (self.initial_q1, "q1 inicial"),
                (self.initial_q2, "q2 inicial"),
                (self.initial_d3, "d3 inicial"),
                (self.final_q1, "q1 final"),
                (self.final_q2, "q2 final"),
                (self.final_d3, "d3 final"),
                (self.duration, "duração"),
                (self.dt, "dt")
            ]
            
            for field, name in fields:
                if not field.text().strip():
                    QMessageBox.warning(self, "Campo Vazio", f"O campo '{name}' deve ser preenchido.")
                    field.setFocus()
                    return False
                
                try:
                    float(field.text())
                except ValueError:
                    QMessageBox.warning(self, "Valor Inválido", f"O campo '{name}' deve conter um número válido.")
                    field.setFocus()
                    return False
            
            # Verificar limites físicos
            d3_initial = float(self.initial_d3.text())
            d3_final = float(self.final_d3.text())
            
            if d3_initial < 0 or d3_initial > self.robot.d3_max:
                QMessageBox.warning(self, "Limite Físico", f"d3 inicial deve estar entre 0 e {self.robot.d3_max}")
                return False
                
            if d3_final < 0 or d3_final > self.robot.d3_max:
                QMessageBox.warning(self, "Limite Físico", f"d3 final deve estar entre 0 e {self.robot.d3_max}")
                return False
            
            # Verificar duração e dt
            duration = float(self.duration.text())
            dt = float(self.dt.text())
            
            if duration <= 0:
                QMessageBox.warning(self, "Valor Inválido", "A duração deve ser maior que zero.")
                return False
                
            if dt <= 0 or dt >= duration:
                QMessageBox.warning(self, "Valor Inválido", "dt deve ser maior que zero e menor que a duração.")
                return False
            
            return True
            
        except Exception as e:
            QMessageBox.critical(self, "Erro de Validação", f"Erro durante validação: {str(e)}")
            return False
    
    def start_simulation(self):
        """Iniciar simulação"""
        try:
            if not hasattr(self, 'trajectory') or not self.trajectory:
                QMessageBox.warning(self, "Aviso", "Calcule uma trajetória primeiro!")
                return
            
            # Configurar thread
            self.simulation_thread.set_trajectory(self.trajectory)
            self.simulation_thread.set_controller(self.use_controller.isChecked())
            
            # Atualizar interface
            self.btn_simulate.setEnabled(False)
            self.btn_stop.setEnabled(True)
            self.btn_calc_trajectory.setEnabled(False)
            self.progress_bar.setVisible(True)
            self.progress_bar.setValue(0)
            
            # Iniciar simulação
            self.simulation_thread.start()
            
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao iniciar simulação: {str(e)}")
            self.reset_simulation_buttons()
    
    def stop_simulation(self):
        """Parar simulação"""
        try:
            self.simulation_thread.stop()
            self.simulation_thread.wait(1000)  # Aguardar até 1 segundo
            
            self.update_status("Simulação parada pelo usuário")
            self.reset_simulation_buttons()
            
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao parar simulação: {str(e)}")
    
    def reset_simulation_buttons(self):
        """Resetar estado dos botões após simulação"""
        self.btn_simulate.setEnabled(True if hasattr(self, 'trajectory') and self.trajectory else False)
        self.btn_stop.setEnabled(False)
        self.btn_calc_trajectory.setEnabled(True)
        self.progress_bar.setVisible(False)
    
    def send_to_robot(self):
        """Enviar para robô real"""
        try:
            if not hasattr(self, 'trajectory') or not self.trajectory:
                QMessageBox.warning(self, "Aviso", "Calcule uma trajetória primeiro!")
                return
            
            # Simular envio para robô
            self.update_status("Preparando envio para robô...")
            
            # Aqui você integraria seu código de comunicação Bluetooth
            # Por exemplo:
            # bluetooth_manager = BluetoothManager()
            # bluetooth_manager.send_trajectory(self.trajectory)
            
            QMessageBox.information(self, "Sucesso", "Trajetória enviada para o robô!\n(Simulação - integre seu código Bluetooth)")
            self.update_status("Trajetória enviada para robô via Bluetooth")
            
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao enviar para robô: {str(e)}")
    
    def update_speed(self, value):
        """Atualizar velocidade"""
        try:
            speed = value / 10.0
            self.speed_label.setText(f"{speed:.1f}x")
            self.simulation_thread.set_speed(speed)
            
        except Exception as e:
            print(f"Erro ao atualizar velocidade: {e}")
    
    def update_robot_position(self, q1, q2, d3):
        """Atualizar posição do robô"""
        try:
            if hasattr(self, 'robot_plot'):
                self.robot_plot.update_robot_position(q1, q2, d3)
            
            self.current_q1, self.current_q2, self.current_d3 = q1, q2, d3
            
        except Exception as e:
            print(f"Erro ao atualizar posição: {e}")
    
    def update_status(self, message):
        """Atualizar status"""
        try:
            timestamp = time.strftime('%H:%M:%S')
            formatted_message = f"[{timestamp}] {message}"
            self.status_text.append(formatted_message)
            
            # Scroll para o final
            scrollbar = self.status_text.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
            
        except Exception as e:
            print(f"Erro ao atualizar status: {e}")
    
    def update_progress(self, value):
        """Atualizar barra de progresso"""
        try:
            self.progress_bar.setValue(value)
            
            # Resetar botões quando a simulação termina
            if value >= 100:
                QTimer.singleShot(1000, self.reset_simulation_buttons)
                
        except Exception as e:
            print(f"Erro ao atualizar progresso: {e}")
    
    def closeEvent(self, event):
        """Evento de fechamento da janela"""
        try:
            # Parar simulação se estiver rodando
            if self.simulation_thread.isRunning():
                self.simulation_thread.stop()
                self.simulation_thread.wait(2000)
            
            event.accept()
            
        except Exception as e:
            print(f"Erro ao fechar aplicação: {e}")
            event.accept()

