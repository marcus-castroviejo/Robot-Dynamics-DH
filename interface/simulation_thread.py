"""
============================================
    Classe SimulationThread
============================================
- Execução em thread separada

Serve para conseguir rodar a simulação em thread separada para não travar a interface
- Controla a parte da simulação em si: controle da velocidade, iniciar, parar, envio de sinais (atualizar posição, ...)

"""
from PyQt6.QtCore import QThread, pyqtSignal


class SimulationThread(QThread):
    """Thread para simulação"""
    # A Thread é iniciada em main_window.py pela função .start()
    
    # Esses sinais vão para a thread principal, conectada em main_window.py
    # Lá os sinais ativam funções para alterar a interface
    position_updated = pyqtSignal(float, float, float)              # Emite sinal: (q1, q2, q3)         # Chama update_robot_position(q1,q2,d3)
    status_updated = pyqtSignal(str)                                # Emite sinal: mensagens            # Chama update_status("text")
    progress_updated = pyqtSignal(int)                              # Emite sinal: progresso [0-100]    # Chama update_progress(x)
    position_data_updated = pyqtSignal(float, float, float, float)  # Emite sinal: (t, q1, q2, d3)      # Chama update_position_graph(t, q1, q2, q3)
    
    def __init__(self):
        super().__init__()
        self.trajectory = []
        self.is_running = False
        self.speed_factor = 0.6
        self.use_controller = False
    
    def set_trajectory(self, trajectory):
        self.trajectory = trajectory
    
    def set_speed(self, speed):
        self.speed_factor = max(0.1, speed)  # Evitar velocidade zero
    
    def set_controller(self, use_controller):
        self.use_controller = use_controller
    
    def run(self):
        """Executar simulação"""
        try:
            self.is_running = True
            self.status_updated.emit("Simulação iniciada")
            
            total_points = len(self.trajectory)
            
            for i, (t, q_joints, qd_joints, qdd_joints) in enumerate(self.trajectory):
                if not self.is_running:
                    break
                    
                q1, q2, d3 = q_joints
                self.position_updated.emit(float(q1), float(q2), float(d3))
                self.position_data_updated.emit(float(t), float(q1), float(q2), float(d3))
                
                # Atualizar progresso
                progress = int((i + 1) / total_points * 100)
                self.progress_updated.emit(progress)
                
                # Status
                self.status_updated.emit(f"Simulando... {progress}% | t={t:.2f}s")
                
                # Sleep baseado na velocidade
                sleep_time = max(10, int(50 / self.speed_factor))
                self.msleep(sleep_time)
            
            if self.is_running:
                self.status_updated.emit("Simulação concluída")
            else:
                self.status_updated.emit("Simulação interrompida")
                
        except Exception as e:
            self.status_updated.emit(f"Erro na simulação: {str(e)}")
        finally:
            self.is_running = False
    
    def stop(self):
        """Parar simulação"""
        self.is_running = False

