"""
=================================================================================================================
                                                Classe SimulationThread
=================================================================================================================
- Execução da Simulação em Thread separada da MainWindow
    - Evita travar a interface
    - Recebe dados de configuração da MainWindow (trajetória desejada, uso de controlador, velocidade de simulação)
    - A Simulação é iniciada através da função .start() chamada na MainWindow -> ativa .run()
    - Envia Sinais para a MainWindow, que atualiza a interface em tempo real
        - Exemplo de sinais: (atualizar posição, status, progresso)

Campos deste código:
["Setup Inicial"]:      Inicialização, Definição dos Sinais
["Configuração"]:       Ajuste da Trajetória desejada, Velocidade de Simulação e Uso de Controlador
["Simulação"]:          Início e Fim da Simulação
"""
from PyQt6.QtCore import QThread, pyqtSignal


class SimulationThread(QThread):
    """Thread para simulação"""
    
    """
    =================================================================================================================
                                                Setup inicial
    =================================================================================================================
    """

    """--------------------------- Sinais de Atualização (update) ---------------------------"""
    # Esses sinais vão para a MainWindow (thread principal), onde ativam funções para atualizar a interface
    position_updated = pyqtSignal(float, float, float, float)   # Emite sinal: (t, q1, q2, d3)      # Chama update_robot_position([qi]) e update_position_graph(t, [qi])
    status_updated = pyqtSignal(str)                            # Emite sinal: mensagens            # Chama update_status("text")
    progress_updated = pyqtSignal(int)                          # Emite sinal: progresso [0-100]    # Chama update_progress(x)
    
    """--------------------------- __init__() ---------------------------"""
    def __init__(self):
        super().__init__()
        self.trajectory = []
        self.is_running = False
        self.speed_factor = 1.0
        self.controller = None
    
    """
    =================================================================================================================
                                                Configuração
    =================================================================================================================
    """

    """--------------------------- Configura a Trajetória desejada ---------------------------"""
    def set_trajectory(self, trajectory):
        self.trajectory = trajectory
    
    """--------------------------- Configura a Velocidade de simulação ---------------------------"""
    def set_speed(self, speed):
        self.speed_factor = max(0.1, speed)  # Evitar velocidade zero
    
    """--------------------------- Configura o uso de Controlador ---------------------------"""
    def set_controller(self, controller):
        self.controller = controller
    
    """
    =================================================================================================================
                                                Simulação
    =================================================================================================================
    """

    """--------------------------- Início da Simulação ---------------------------"""
    def run(self):
        """Executar simulação"""
        # A Simulação é iniciada em main_window.py pela função .start()
        try:
            self.is_running = True
            self.status_updated.emit("Simulação iniciada")
            
            total_points = len(self.trajectory)
            
            # Itera sobre os pontos da trajetória e emite sinais para a Main_Window, que atualiza os plots
            # (!!!) O controle vai ter que entrar aqui no loop
            for i, trajectory in enumerate(self.trajectory):
                t, q_traj, qd_traj, qdd_traj = trajectory

                if not self.is_running:
                    break
                
                # Controlador
                if self.controller:
                    errors, q_joints, qd_joints, qdd_joints, tau = self.controller.control_position(*trajectory[1:])
                else:
                    errors, q_joints, qd_joints, qdd_joints, tau = 0.0, q_traj, qd_traj, qdd_traj, 0.0
                
                # print(t, q_joints)

                # Atualizar a posição das juntas
                q1, q2, d3 = q_joints
                self.position_updated.emit(float(t), float(q1), float(q2), float(d3))
                
                # Atualizar progresso
                progress = int((i + 1) / total_points * 100)
                self.progress_updated.emit(progress)
                
                # Atualizar o Status (log)
                self.status_updated.emit(f"Simulando... {progress}% | t={t:.2f}s")
                
                # Sleep baseado na velocidade
                sleep_time = max(10, int(100 / self.speed_factor))          # 100 fica legal
                self.msleep(sleep_time)
            
            if self.is_running:
                self.status_updated.emit("Simulação concluída")
            else:
                self.status_updated.emit("Simulação interrompida")
                
        except Exception as e:
            self.status_updated.emit(f"Erro na simulação: {str(e)}")
        finally:
            self.is_running = False
    
    # def run(self):
    #     """Executar simulação"""
    #     # A Simulação é iniciada em main_window.py pela função .start()
    #     try:
    #         self.is_running = True
    #         self.status_updated.emit("Simulação iniciada")
            
    #         total_points = len(self.trajectory)
            
    #         # Itera sobre os pontos da trajetória e emite sinais para a Main_Window, que atualiza os plots
    #         # (!!!) O controle vai ter que entrar aqui no loop
    #         for i, (t, q_joints, qd_joints, qdd_joints) in enumerate(self.trajectory):
    #             if not self.is_running:
    #                 break
                
    #             # Atualizar a posição das juntas
    #             q1, q2, d3 = q_joints
    #             self.position_updated.emit(float(t), float(q1), float(q2), float(d3))
                
    #             # Atualizar progresso
    #             progress = int((i + 1) / total_points * 100)
    #             self.progress_updated.emit(progress)
                
    #             # Atualizar o Status (log)
    #             self.status_updated.emit(f"Simulando... {progress}% | t={t:.2f}s")
                
    #             # Sleep baseado na velocidade
    #             sleep_time = max(10, int(100 / self.speed_factor))          # 100 fica legal
    #             self.msleep(sleep_time)
            
    #         if self.is_running:
    #             self.status_updated.emit("Simulação concluída")
    #         else:
    #             self.status_updated.emit("Simulação interrompida")
                
    #     except Exception as e:
    #         self.status_updated.emit(f"Erro na simulação: {str(e)}")
    #     finally:
    #         self.is_running = False
    
    """--------------------------- Fim da Simulação ---------------------------"""
    def stop(self):
        """Parar simulação"""
        self.is_running = False
