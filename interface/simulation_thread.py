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
    - Usa CommunicationManager para se comunicar com a ESP32

Campos deste código:
["Setup Inicial"]:      Inicialização, Definição dos Sinais
["Configuração"]:       Ajuste da Trajetória desejada, Velocidade de Simulação e Uso de Controlador
["Simulação"]:          Início e Fim da Simulação
"""
from PyQt6.QtCore import QThread, pyqtSignal
import numpy as np
import traceback
from typing import Optional


class SimulationThread(QThread):
    """Thread para simulação com comunicação ESP32"""
    
    """
    =================================================================================================================
                                                Setup inicial
    =================================================================================================================
    """

    """--------------------------- Sinais de Atualização (update) ---------------------------"""
    # Sinais para a MainWindow (thread principal) atualizar interface
    position_updated = pyqtSignal(float, list, list, list, list, list, list, list)
    status_updated = pyqtSignal(str)
    progress_updated = pyqtSignal(int)

    """--------------------------- __init__() ---------------------------"""
    def __init__(self):
        super().__init__()
        
        # Configuração da simulação
        self.trajectory = []
        self.is_running = False
        self.is_paused = False
        self.speed_factor = 1.0
        self.controller = None
        
        # Gerenciador de comunicação (será injetado externamente)
        self.comm_manager = None
        
        # Estado da garra
        self.current_gripper_value = 0
    
    """
    =================================================================================================================
                                                Configuração
    =================================================================================================================
    """

    """--------------------------- Configura a Trajetória desejada ---------------------------"""
    def set_trajectory(self, trajectory):
        """Define a trajetória para simulação"""
        self.trajectory = trajectory
    
    """--------------------------- Configura a Velocidade de simulação ---------------------------"""
    def set_speed(self, speed):
        """Define velocidade de simulação"""
        self.speed_factor = max(0.1, speed)  # Evitar velocidade zero
    
    """--------------------------- Configura o uso de Controlador ---------------------------"""
    def set_controller(self, controller):
        """Define o controlador a ser usado"""
        self.controller = controller
    
    """--------------------------- Configura o CommunicationManager ---------------------------"""
    def set_communication_manager(self, comm_manager):
        """Injeta o gerenciador de comunicação"""
        self.comm_manager = comm_manager
    
    """--------------------------- Atualiza valor da garra ---------------------------"""
    def set_current_gripper_value(self, value: int):
        """Atualiza o valor desejado da garra (recebe em graus)"""
        self.current_gripper_value = value
    
    """
    =================================================================================================================
                                                Simulação
    =================================================================================================================
    """

    """--------------------------- Início da Simulação ---------------------------"""
    def run(self):
        """
        Executar simulação
        
        Ciclo: Pedir medição → Aguardar → Calcular controle → Enviar comando
        """
        try:
            self.is_running = True
            self.status_updated.emit("Simulação iniciada")
            total_points = len(self.trajectory)
            controller_mode = controller_mode

            if controller_mode not in ('Torque Calculado', 'PID', 'PID (Baixo Nível)', 'Simulação'):
                self.status_updated.emit(f"Erro na simulação: Controlador não definido ({controller_mode})")
                return

            # Comunicação para modos (exceto Simulação)
            if controller_mode != 'Simulação':
                # Verifica se há comunicação
                if self.comm_manager is None or not self.comm_manager.is_connected():
                    raise RuntimeError("ESP32 não conectada. Inicie o servidor primeiro.")

                # Enviar Ganhos: BAIXO NIVEL
                if controller_mode == "PID (Baixo Nível)":
                    self.status_updated.emit("Enviando ganhos para ESP32...")

                    Kp = np.diag(self.controller.Kp).tolist()
                    Kd = np.diag(self.controller.Kd).tolist()
                    Ki = np.diag(self.controller.Ki).tolist()

                    if not self.comm_manager.send_gains(Kp, Kd, Ki):
                        self.status_updated.emit("Aviso: falha ao enviar ganhos")
                    else:
                        self.status_updated.emit("Ganhos configurados na ESP32")
                    
                    # Pequena pausa para ESP32 processar
                    self.msleep(100)

                # ===== SINCRONIZAÇÃO INICIAL =====
                try:
                    q0 = list(np.asarray(self.trajectory[0][1], dtype=float).reshape(3))
                except Exception:
                    q0 = [0.0, 0.0, 0.0]

                self.status_updated.emit("Sincronizando com ESP32...")
                
                # Usa CommunicationManager para sincronizar
                synced = self.comm_manager.synchronize_initial_position(q0, self.current_gripper_value)
                
                if not synced:
                    self.status_updated.emit("Aviso: sincronização não confirmada; prosseguindo")
                else:
                    self.status_updated.emit("ESP32 sincronizada")
            
            # Configura controlador
            if self.controller:
                self.controller.set_trajectory(self.trajectory)

            # ===== LOOP PRINCIPAL =====
            for i, trajectory_point in enumerate(self.trajectory):
                # Verifica pausa
                while self.is_paused and self.is_running:
                    self.msleep(50)
                    
                if not self.is_running:
                    break

                t, q_traj, qd_traj, qdd_traj = trajectory_point
                
                # === SIMULACAO ===
                if controller_mode == 'Simulação':
                    # Sem controlador: usa trajetória direta
                    q_command = None
                    q_real = q_traj.copy()
                    qd_real = qd_traj.copy()
                    qdd_real = qdd_traj.copy()
                    e_pos = [0.0, 0.0, 0.0]
                    e_vel = [0.0, 0.0, 0.0]
                    e_acc = [0.0, 0.0, 0.0]
                    tau = [0.0, 0.0, 0.0]
                
                else:
                    # 0) ENVIAR COMANDO/REFERÊNCIA (BAIXO NIVEL)
                    # if controller_mode == 'PID (Baixo Nível)':
                    #     # Envia trajetória completa para ESP32 executar PID
                    #     q_ref = list(np.asarray(q_traj, dtype=float).reshape(3))
                    #     qd_ref = list(np.asarray(qd_traj, dtype=float).reshape(3))
                    #     qdd_ref = list(np.asarray(qdd_traj, dtype=float).reshape(3))
                    #     self.comm_manager.send_reference_pid(q_ref, qd_ref, qdd_ref, self.current_gripper_value)
                    
                    # 1) SOLICITAR MEDIÇÃO
                    self.comm_manager.request_measurement()
                    
                    # 2) AGUARDAR MEDIÇÃO
                    timeout_ms = int(max(20, 100 / self.speed_factor))
                    
                    if not self.comm_manager.wait_for_measurement(timeout_ms):
                        self.status_updated.emit("Aguardando medição da ESP32...")
                        self.comm_manager.request_measurement()
                        if not self.comm_manager.wait_for_measurement(timeout_ms):
                            self.status_updated.emit("Timeout ao aguardar medição")
                            continue
                    
                    # 3) OBTER MEDIÇÃO
                    measurement = self.comm_manager.get_last_measurement()
                    
                    if measurement is None or not measurement.is_valid():
                        self.status_updated.emit("Medição inválida recebida")
                        continue
                    
                    # 4) PROCESSAR CONTROLADOR
                    if controller_mode == 'PID (Baixo Nível)':
                        q_real = measurement.q
                        qd_real = qd_traj           # Aproximação para plotagem
                        qdd_real = qdd_traj
                        e_pos = [q_traj[j] - q_real[j] for j in range(3)]
                        e_vel = [0.0, 0.0, 0.0]
                        e_acc = [0.0, 0.0, 0.0]
                        tau = [0.0, 0.0, 0.0]
                        
                    else:
                        try:
                            # Entrega medição ao controlador
                            self.controller.set_measurement(measurement.q, t=measurement.timestamp)
                            
                            # Calcula controle
                            (
                                q_command,   # Comando de posição calculado
                                q_real,      # Posição real medida
                                qd_real,     # Velocidade calculada
                                qdd_real,    # Aceleração calculada
                                e_pos,       # Erro de posição
                                e_vel,       # Erro de velocidade
                                e_acc,       # Erro de aceleração
                                tau          # Torque calculado
                            ) = self.controller.update_control_position()
                        
                        except RuntimeError as ex:
                            if str(ex) == "sem_medidoes":
                                self.status_updated.emit("Aguardando primeira medição válida...")
                                continue
                            else:
                                raise
                    
                        # 5) ENVIAR COMANDO
                        # TODO: Trocar q_traj por q_command quando tudo estiver funcionando
                        q_to_send = list(np.asarray(q_command, dtype=float).reshape(3))
                        self.comm_manager.send_reference(q_to_send, self.current_gripper_value)
                
                # 6) ATUALIZAR INTERFACE
                self.position_updated.emit(
                    float(t),
                    list(q_real),
                    list(qd_real),
                    list(qdd_real),
                    list(e_pos),
                    list(e_vel),
                    list(e_acc),
                    list(tau)
                )
                
                # 7) PROGRESSO
                progress = int((i + 1) / total_points * 100)
                self.progress_updated.emit(progress)
                self.status_updated.emit(f"Simulando... {progress}% | t={t:.2f}s")

                # Sleep para controlar velocidade da animação
                self.msleep(max(10, int(100 / self.speed_factor)))

            # Fim do loop
            if self.is_running:
                self.status_updated.emit("Simulação concluída")
            else:
                self.status_updated.emit("Simulação interrompida")
                
        except Exception as e:
            print("--- TRACEBACK COMPLETO DO ERRO ---")
            traceback.print_exc()
            print("----------------------------------")
            self.status_updated.emit(f"Erro na simulação: {str(e)}")
        finally:
            self.is_running = False
            self.is_paused = False

    """--------------------------- Fim da Simulação ---------------------------"""
    def stop(self):
        """Parar simulação"""
        self.is_running = False

    """--------------------------- Pausar Simulação ---------------------------"""
    def pause(self):
        """Pausar a simulação"""
        self.is_paused = True
    
    """--------------------------- Retomar Simulação ---------------------------"""
    def resume(self):
        """Retomar a simulação"""
        self.is_paused = False