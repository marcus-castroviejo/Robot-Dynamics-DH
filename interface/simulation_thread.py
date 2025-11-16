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
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot, QEventLoop, QTimer
import numpy as np
from time import perf_counter
import random
import traceback


class SimulationThread(QThread):
    """Thread para simulação"""
    
    """
    =================================================================================================================
                                                Setup inicial
    =================================================================================================================
    """

    """--------------------------- Sinais de Atualização (update) ---------------------------"""
    # Esses sinais vão para a MainWindow (thread principal), onde ativam funções para atualizar a interface
    position_updated = pyqtSignal(float, list, list, list, list, list, list, list)
    status_updated = pyqtSignal(str)
    progress_updated = pyqtSignal(int)

    # === Sinais de integração com a comunicação ===
    tx_json = pyqtSignal(dict)
    
    # (Recebe: Posição real, Posição real da garra, Tempo)
    ingest_meas_sig = pyqtSignal(list, int, float)


    
    """--------------------------- __init__() ---------------------------"""
    def __init__(self):
        super().__init__()
        self.trajectory = []
        self.is_running = False
        self.is_paused = False
        self.speed_factor = 1.0
        self.controller = None

        self.last_meas = None     
        self.last_meas_t = 0.0

        # Guarda o valor do slider (enviado para a ESP)
        self.current_gripper_value = 0 
        # Guarda o valor real da garra (recebido da ESP)
        self.last_gripper_real = 0 

        self.ingest_meas_sig.connect(self.ingest_meas)

        # Adicionar o dado do gripper para receber e envia-lo durante a simulação
        self.next_gripper_command = 0
        self.current_gripper_value = 0

        self._rx_seq = 0
        self.sync_timeout_s = 2.0
        self.sync_tol = 1e-3


    
    """
    =================================================================================================================
                                                Configuração
    =================================================================================================================
    """

    # <<< MODIFICADO: Slot atualizado para aceitar q, qd, qdd e i
    @pyqtSlot(list, int, float)
    def ingest_meas(self, meas_q, meas_gripper, t):
        """Recebe os dados medidos da MainWindow."""
        self.last_meas = meas_q
        self.last_gripper_real = meas_gripper
        self.last_meas_t = t
        self._rx_seq += 1

    def set_current_gripper_value(self, value: int):
        """
        Slot chamado pela main_window para atualizar o 
        valor desejado da garra.
        """
        self.current_gripper_value = value
    
    
    def _wait_for_new_meas(self, prev_seq, timeout_ms=100):
        # verifica se chegou algun dado novo
        if self._rx_seq > prev_seq:
            return True

        loop = QEventLoop()
        hit = {"ok": False}

        def _on_new(_meas_q, _meas_gripper, _t):
            hit["ok"] = True
            loop.quit()

        # ouvir a próxima medição
        self.ingest_meas_sig.connect(_on_new)
        timer = QTimer()
        timer.setSingleShot(True)
        timer.timeout.connect(loop.quit)
        timer.start(timeout_ms)

        loop.exec()

        try:
            self.ingest_meas_sig.disconnect(_on_new)
        except TypeError:
            pass

        return hit["ok"]


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
        """Executar simulação (Lógica V6: Pedir -> Esperar -> Calcular -> Enviar)"""
        try:
            self.is_running = True
            self.status_updated.emit("Simulação iniciada")
            total_points = len(self.trajectory)

            if self.controller:
                self.controller.set_trajectory(self.trajectory)

            # ===== SYNC COM A ESP32 (garantir que a 1ª medição = q0) =====
            try:
                q0 = list(np.asarray(self.trajectory[0][1], dtype=float).reshape(3))
            except Exception:
                q0 = [0.0, 0.0, 0.0]

            self.status_updated.emit("Sincronizando com ESP…")
            
            # Comando inicial de sync (envia q0 e a garra)
            command_to_send = {
                "cmd": "set_ref",
                "q_cmd": q0, # Envia a posição inicial como comando
                "gripper": self.current_gripper_value
            }

            t0 = perf_counter()
            synced = False
            while self.is_running and (perf_counter() - t0) < self.sync_timeout_s:
                self.tx_json.emit(command_to_send) # Envia o comando de sync
                self.tx_json.emit({"cmd": "get_meas"}) # Pede a medição
                
                # Espera 100ms por uma resposta
                seq_before = self._rx_seq
                if not self._wait_for_new_meas(seq_before, timeout_ms=100):
                    continue # (Timeout, tenta o loop 'while' de novo)

                # Resposta chegou! Checa o erro.
                if self.last_meas is not None:
                    try:
                        err = np.linalg.norm(np.asarray(self.last_meas, dtype=float).reshape(3) - q0)
                        if err < self.sync_tol:
                            synced = True
                            break # Sincronizado! Sai do loop 'while'
                    except Exception:
                        pass
            
            if not synced:
                self.status_updated.emit("Aviso: sync não confirmou; seguindo mesmo assim.")
            else:
                self.status_updated.emit("ESP sincronizada.")
            
            for i, trajectory in enumerate(self.trajectory):
                while self.is_paused and self.is_running:
                    self.msleep(50)
                if not self.is_running:
                    break

                t, q_traj, qd_traj, qdd_traj = trajectory

                # 1) PEDIR MEDIÇÃO (do passo anterior)
                #    (O comando 'set_ref' já foi enviado no fim do loop anterior
                #     ou no sync, agora só pedimos a medição)
                self.tx_json.emit({"cmd": "get_meas"})

                # 2) esperar medição
                seq_before = self._rx_seq
                self.tx_json.emit({"cmd": "get_meas"}) # Pede de novo por segurança

                while self.is_running and not self._wait_for_new_meas(seq_before, timeout_ms=int(max(20, 100 / self.speed_factor))):
                    self.status_updated.emit("Aguardando medição da ESP…")
                    self.tx_json.emit({"cmd": "get_meas"})
                
                # 3) Entregar o dado para o controlador
                if self.controller and self.last_meas is not None:
                    try:
                        self.controller.set_measurement(self.last_meas, t=None)
                    except Exception as e:
                        print(f"Erro ao chamar set_measurement: {e}")
                        pass

                # 4) Calculo do novo command
                try:
                    (
                        q_command,  # O novo comando de posição
                        q_real,     # A posição real usada
                        qd_real,
                        qdd_real,   
                        e_pos, e_vel, e_acc, tau
                    ) = self.controller.update_control_position()
                
                except RuntimeError as ex:
                    if str(ex) == "sem_medidoes":
                        self.status_updated.emit("Medição ainda não disponível; aguardando…")
                        continue
                    else:
                        raise
                #Só para testes
                q_traj_deste_passo = list(np.asarray(q_traj, dtype=float).reshape(3))
                # 5) Enviar o comando depois tem que trocar para o q_command
                command_to_send = {"cmd": "set_ref", "q_cmd": list(q_traj_deste_passo), "gripper": self.current_gripper_value}
                self.tx_json.emit(command_to_send)

                # 6) Envia para UI
                self.position_updated.emit(float(t), list(q_real), list(qd_real), list(qdd_real), list(e_pos), list(e_vel), list(e_acc), list(tau))
                
                # 7) Atualizar loop
                progress = int((i + 1) / total_points * 100)
                self.progress_updated.emit(progress)
                self.status_updated.emit(f"Simulando... {progress}% | t={t:.2f}s")

                self.msleep(max(10, int(100 / self.speed_factor)))

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

    """--------------------------- Em construção (!!!!) ---------------------------"""
    def pause(self):
        """Pausar a Simulação"""
        self.is_paused = True
    
    """--------------------------- Em construção (!!!!) ---------------------------"""
    def resume(self):
        """Pausar a Simulação"""
        self.is_paused = False