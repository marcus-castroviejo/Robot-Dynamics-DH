"""
============================================
    Classe RobotControlInterface 
============================================
- Interface principal
Toda a definição dos paineis e campos da interface
Controle do simulação
Validação das entradas, atualização dos dados da simulação
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
from trajectory import TrajectoryGeneratorRRP
from visualization import RobotPlot3D
from .simulation_thread import SimulationThread


class RobotControlInterface(QMainWindow):
    """Interface principal"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Controle CocoaBot")
        self.setup_responsive_window()
        
        # Inicializar componentes
        self.init_components()
        self.init_ui()
        self.setup_connections()
        
        # Posição inicial padrão
        self.current_q1 = np.deg2rad(0.0)
        self.current_q2 = np.deg2rad(80.0)
        self.current_d3 = .03
        
        # Atualizar visualização inicial
        self.update_robot_position(self.current_q1, self.current_q2, self.current_d3)

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
    
    def create_control_panel(self):
        """Painel de controles"""
        panel = QFrame()
        panel.setFrameStyle(QFrame.Shape.StyledPanel)

        # Tamanho do Painel de Controle (minimo e maximo)
        panel.setMinimumWidth(300)
        panel.setMaximumWidth(350)
        
        layout = QVBoxLayout(panel)
        layout.setSpacing(5)                        # Espaço entre os Campos
        # layout.setContentsMargins(8, 8, 8, 8)     # Margens
        
        # Título
        title = QLabel("Controle CocoaBot")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("QLabel { color: #2E7D32; padding: 10px; }")
        layout.addWidget(title)
        
        # Posição inicial e final
        layout.addWidget(self.create_positions_group())
        # Parâmetros de trajetória
        layout.addWidget(self.create_trajectory_group())
        # Controles de simulação
        layout.addWidget(self.create_simulation_group())
        # Status
        layout.addWidget(self.create_status_group())
        
        layout.addStretch()
        return panel
    
    def create_positions_group(self):
        """Posições inicial e final"""
        group = QGroupBox("Posições")
        layout = QGridLayout(group)
        layout.setVerticalSpacing(4)
        layout.setHorizontalSpacing(5)
        
        # Validador
        double_validator = QDoubleValidator()
        double_validator.setDecimals(2)
        double_validator.setLocale(QLocale(QLocale.Language.English, QLocale.Country.UnitedStates))

        # Campos
        self.initial_q1 = QLineEdit("0.0")
        self.initial_q2 = QLineEdit("80.0")
        self.initial_d3 = QLineEdit("3.0")
        self.final_q1 = QLineEdit("90.0")
        self.final_q2 = QLineEdit("135.0")
        self.final_d3 = QLineEdit("8.0")
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
        
        # Headers
        init_label = QLabel("Inicial")
        final_label = QLabel("Final")
        layout.addWidget(init_label, 0, 1)
        layout.addWidget(final_label, 0, 2)
        
        # Linhas das juntas
        q1_label = QLabel("q1 [º]:")
        layout.addWidget(q1_label, 1, 0)
        layout.addWidget(self.initial_q1, 1, 1)
        layout.addWidget(self.final_q1, 1, 2)
        
        q2_label = QLabel("q2 [º]:")
        layout.addWidget(q2_label, 2, 0)
        layout.addWidget(self.initial_q2, 2, 1)
        layout.addWidget(self.final_q2, 2, 2)
        
        d3_label = QLabel("d3 [cm]:")
        layout.addWidget(d3_label, 3, 0)
        layout.addWidget(self.initial_d3, 3, 1)
        layout.addWidget(self.final_d3, 3, 2)
        
        return group
    
    def create_trajectory_group(self):
        """Parâmetros de trajetória"""
        group = QGroupBox("Trajetória")
        layout = QGridLayout(group)
        layout.setVerticalSpacing(4)
        
        # Validador
        double_validator = QDoubleValidator()
        double_validator.setDecimals(0)
        double_validator.setLocale(QLocale(QLocale.Language.English, QLocale.Country.UnitedStates))
        double_validator.setBottom(1)
        # Duração e dt
        self.duration = QLineEdit("5.0")
        self.dt = QLineEdit("30")
        self.duration.setValidator(double_validator)
        self.dt.setValidator(double_validator)
        self.duration.setMaximumHeight(35)
        self.dt.setMaximumHeight(35)
        
        # Labels
        dur_label = QLabel("Duração [s]:")
        dt_label = QLabel("dt [ms]:")
        
        # Adicionar os Widgets
        layout.addWidget(dur_label, 0, 0)
        layout.addWidget(self.duration, 0, 1)
        layout.addWidget(dt_label, 1, 0)
        layout.addWidget(self.dt, 1, 1)
        
        return group
    
    def create_simulation_group(self):
        """Controles de simulação"""
        group = QGroupBox("Simulação")
        layout = QVBoxLayout(group)
        layout.setSpacing(4)
        
        # Checkbox Controlador
        self.use_controller = QCheckBox("Usar controlador Torque Calculado")
        layout.addWidget(self.use_controller)
        
        # Velocidade
        speed_layout = QHBoxLayout()
        speed_layout.setSpacing(4)
        speed_layout.addWidget(QLabel("Velocidade:"))
        # Slider
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(1, 50)
        self.speed_slider.setValue(10)
        speed_layout.addWidget(self.speed_slider)
        # Label
        self.speed_label = QLabel("0.6x")
        self.speed_label.setFont(QFont("Arial", 11))
        self.speed_label.setMinimumWidth(40)
        speed_layout.addWidget(self.speed_label)
        layout.addLayout(speed_layout)
        
        # Barra de progresso
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)
        
        # Botões
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
    
    def create_status_group(self):
        """Grupo de status"""
        group = QGroupBox("Status")
        layout = QVBoxLayout(group)
        # layout.setContentsMargins(5, 5, 5, 5)
        
        self.status_text = QTextEdit()
        self.status_text.setMaximumHeight(200)
        self.status_text.setReadOnly(True)
        self.status_text.setFont(QFont("Monaco, Consolas, Monospace", 8))
        
        layout.addWidget(self.status_text)
        return group
    
    def create_visualization_panel(self):
        """Painel de visualização"""
        panel = QFrame()
        panel.setFrameStyle(QFrame.Shape.StyledPanel)
        layout = QVBoxLayout(panel)
        # layout.setContentsMargins(5, 5, 5, 5)
        
        # Título
        title = QLabel("Visualização 3D")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)
        
        # Plot 3D
        try:
            self.robot_plot = RobotPlot3D()
            layout.addWidget(self.robot_plot)
        except Exception as e:
            error_label = QLabel(f"Erro: {str(e)}")
            error_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(error_label)
        
        return panel



    # ======== MÉTODOS FUNCIONAIS ========



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
            self.simulation_thread.position_data_updated.connect(self.update_position_graph)
            
        except Exception as e:
            self.update_status(f"Erro ao configurar conexões: {str(e)}")
    
    def calculate_trajectory(self):
        """Calcular trajetória"""
        try:
            if not self.validate_inputs():
                return
            
            # Pega a informação que está no campo Posições
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
            
            tf = float(self.duration.text())
            dt = float(self.dt.text()) / 1000
            
            self.trajectory = self.trajectory_gen.generate_trajectory(q0, qf, tf, dt)
            
            trajectory_points = []
            for _, q_joints, _, _ in self.trajectory:
                pos = self.robot.forward_kinematics(*q_joints)
                trajectory_points.append(pos)
            
            # Adiciona nos plots as posições de Target
            if hasattr(self, 'robot_plot'):
                self.robot_plot.update_trajectory(trajectory_points)
                x_final, y_final, z_final = self.robot.forward_kinematics(*qf)
                self.robot_plot.set_target_position(x_final, y_final, z_final)
            
            self.btn_simulate.setEnabled(True)
            self.update_status(f"Trajetória: {len(self.trajectory)} pts, {tf}s")
            
        except ValueError as e:
            QMessageBox.warning(self, "Entrada Inválida", f"Verifique os valores: {str(e)}")
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao calcular trajetória: {str(e)}")
    
    def validate_inputs(self):
        """Validar entradas"""
        try:
            fields = [
                (self.initial_q1, "q1 inicial"), (self.initial_q2, "q2 inicial"), (self.initial_d3, "d3 inicial"),
                (self.final_q1, "q1 final"), (self.final_q2, "q2 final"), (self.final_d3, "d3 final"),
                (self.duration, "duração"), (self.dt, "dt")
            ]
            
            for field, name in fields:
                if not field.text().strip():
                    QMessageBox.warning(self, "Campo Vazio", f"Preencha '{name}'")
                    field.setFocus()
                    return False
                
                try:
                    float(field.text())
                except ValueError:
                    QMessageBox.warning(self, "Valor Inválido", f"'{name}' deve ser numérico")
                    field.setFocus()
                    return False
            
            # D3: limitante (deve estar dentro dos limites)
            d3_initial = float(self.initial_d3.text())
            d3_final = float(self.final_d3.text())
            d3_max = 100*self.robot.d3_max
            
            if d3_initial < 0 or d3_initial > d3_max:
                QMessageBox.warning(self, "Limite Físico", f"d3 inicial: 0.0 a {d3_max} cm")
                return False
                
            if d3_final < 0 or d3_final > d3_max:
                QMessageBox.warning(self, "Limite Físico", f"d3 final: 0.0 a {d3_max} cm")
                return False
            
            # Duração: limitante (deve estar dentro dos limites)
            duration = float(self.duration.text())
            dt = float(self.dt.text()) / 1000
            
            if duration <= 0:
                QMessageBox.warning(self, "Valor Inválido", "Duração > 0")
                return False
                
            if dt <= 0 or dt >= duration:
                QMessageBox.warning(self, "Valor Inválido", "0 < dt < duração")
                return False
            
            # Posições: só Warning (pode simular fora dos limites)
            q1_initial = float(self.initial_q1.text())
            q2_initial = float(self.initial_q2.text())
            q1_final = float(self.final_q1.text())
            q2_final = float(self.final_q2.text())
            q1_min, q1_max = np.rad2deg(np.array(self.robot.J1_range, float))
            q2_min, q2_max = np.rad2deg(np.array(self.robot.J2_range, float))

            if q1_initial < q1_min or q1_initial > q1_max:
                QMessageBox.warning(self, "Limite Físico", f"q1 inicial: {q1_min} a {q1_max} graus")
            
            if q1_final < q1_min or q1_final > q1_max:
                QMessageBox.warning(self, "Limite Físico", f"q1 final: {q1_min} a {q1_max} graus")
            
            if q2_initial < q2_min or q2_initial > q2_max:
                QMessageBox.warning(self, "Limite Físico", f"q2 inicial: {q2_min} a {q2_max} graus")
            
            if q2_final < q2_min or q2_final > q2_max:
                QMessageBox.warning(self, "Limite Físico", f"q2 final: {q2_min} a {q2_max} graus")

            return True
            
        except Exception as e:
            QMessageBox.critical(self, "Erro de Validação", str(e))
            return False
    
    def start_simulation(self):
        """Iniciar simulação"""
        try:
            if not hasattr(self, 'trajectory') or not self.trajectory:
                QMessageBox.warning(self, "Aviso", "Calcule trajetória primeiro!")
                return
            
            if hasattr(self, 'robot_plot'):
                self.robot_plot.reset_position_graph()
            
            # Passa a trajetória e o controle para o thread
            self.simulation_thread.set_trajectory(self.trajectory)
            self.simulation_thread.set_controller(self.use_controller.isChecked())
            
            # Ativa/desativa os botões
            self.btn_simulate.setEnabled(False)
            self.btn_stop.setEnabled(True)
            self.btn_calc_trajectory.setEnabled(False)
            self.progress_bar.setVisible(True)
            self.progress_bar.setValue(0)
            
            # Inicia a simulação na thread
            self.simulation_thread.start()
            
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao iniciar simulação: {str(e)}")
            self.reset_simulation_buttons()
    
    def stop_simulation(self):
        """Parar simulação"""
        try:
            self.simulation_thread.stop()
            self.simulation_thread.wait(1000)
            self.update_status("Simulação parada")
            self.reset_simulation_buttons()
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao parar simulação: {str(e)}")
    
    def reset_simulation_buttons(self):
        """Resetar botões após simulação"""
        self.btn_simulate.setEnabled(True if hasattr(self, 'trajectory') and self.trajectory else False)
        self.btn_stop.setEnabled(False)
        self.btn_calc_trajectory.setEnabled(True)
        self.progress_bar.setVisible(False)
    
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
            scrollbar = self.status_text.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
        except Exception as e:
            print(f"Erro ao atualizar status: {e}")
    
    def update_progress(self, value):
        """Atualizar progresso"""
        try:
            self.progress_bar.setValue(value)
            if value >= 100:
                QTimer.singleShot(1000, self.reset_simulation_buttons)
        except Exception as e:
            print(f"Erro ao atualizar progresso: {e}")
    
    def update_position_graph(self, t, q1, q2, d3):
        """Atualizar gráfico de posição"""
        try:
            if hasattr(self, 'robot_plot'):
                self.robot_plot.update_position_graph(t, q1, q2, d3)
        except Exception as e:
            print(f"Erro ao atualizar gráfico de posição: {e}")
    
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
