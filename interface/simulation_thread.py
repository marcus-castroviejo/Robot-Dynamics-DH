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
    position_updated = pyqtSignal(float, list, list, list, list, list, list, list, list)
    status_updated = pyqtSignal(str)
    progress_updated = pyqtSignal(int)

    # === Sinais de integração com a comunicação ===
    tx_json = pyqtSignal(dict)
    
    # <<< MODIFICADO: O sinal de ingestão agora deve carregar q, qd, e qdd
    # (V2) ingest_meas_sig = pyqtSignal(list, float)
    ingest_meas_sig = pyqtSignal(list, list, list, list, float)  # (q, qd, qdd, t)


    
    """--------------------------- __init__() ---------------------------"""
    def __init__(self):
        super().__init__()
        self.trajectory = []
        self.is_running = False
        self.is_paused = False
        self.speed_factor = 1.0
        self.controller = None

        # estado de rede / medições
        self.last_meas = None     
        self.last_meas_t = 0.0
        self.last_meas_qdd = None
        self.last_meas_qdd2 = None
        self.last_meas_i = None

        # Velocidade a ser calculado com integração númerica.
        self.qd_calculado = np.array([0.0, 0.0, 0.0])
        
        # conecta o sinal de ingestão ao slot local (thread-safe)
        self.ingest_meas_sig.connect(self.ingest_meas)

        self._rx_seq = 0
        self.sync_timeout_s = 2.0
        self.sync_tol = 1e-3


    
    """
    =================================================================================================================
                                                Configuração
    =================================================================================================================
    """

    # <<< MODIFICADO: Slot atualizado para aceitar q, qd, qdd e i
    @pyqtSlot(list, list, list, list, float)
    def ingest_meas(self, meas_q, meas_qdd, meas_qdd2, meas_i, t):
        """Recebe os dados medidos da MainWindow."""
        self.last_meas = meas_q
        self.last_meas_qdd = meas_qdd 
        self.last_meas_qdd2 = meas_qdd2   
        self.last_meas_i = meas_i
        self.last_meas_t = t
        self._rx_seq += 1
    
    
    def _wait_for_new_meas(self, prev_seq, timeout_ms=100):
        # verifica se chegou algun dado novo
        if self._rx_seq > prev_seq:
            return True

        loop = QEventLoop()
        hit = {"ok": False}

        def _on_new(__q, _qdd, _qdd2, _i, _t):
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
        """Executar simulação"""
        try:
            self.is_running = True
            self.status_updated.emit("Simulação iniciada")

            total_points = len(self.trajectory)

            if self.controller:
                self.controller.set_trajectory(self.trajectory)


                # Zera a velocidade no inicio da simulação
                self.qd_calculado = np.array([0.0, 0.0, 0.0])

            # ===== SYNC COM A ESP32 (garantir que a 1ª medição = q0) =====
            try:
                q0 = list(np.asarray(self.trajectory[0][1], dtype=float).reshape(3))
                qdd0 = list(np.asarray(self.trajectory[0][3], dtype=float).reshape(3))
            except Exception:
                q0 = [0.0, 0.0, 0.0]
                qdd0 = [0.0, 0.0, 0.0]

            self.status_updated.emit("Sincronizando com ESP…")
            
            # <<< MODIFICADO: Envia q, e qdd no 'set_ref' de sync
            # self.tx_json.emit({"cmd": "set_ref", "q": [float(q0[0]), float(q0[1]), float(q0[2])]})
            self.tx_json.emit({"cmd": "set_ref", "q": q0, "qdd": qdd0})

            t0 = perf_counter()
            synced = False
            while self.is_running and (perf_counter() - t0) < self.sync_timeout_s:
                # pedir uma medição
                self.tx_json.emit({"cmd": "get_meas"})
                # aguarda até 100 ms em fatias de 10 ms
                for _ in range(10):
                    if self.last_meas is not None:
                        try:
                            # (O sync check ainda compara só a posição, o que é OK)
                            err = np.linalg.norm(np.asarray(self.last_meas, dtype=float).reshape(3) - q0)
                            if err < self.sync_tol:
                                synced = True
                                break
                        except Exception:
                            pass
                    self.msleep(10)
                if synced:
                    break

            if not synced:
                self.status_updated.emit("Aviso: sync não confirmou; seguindo mesmo assim.")
            else:
                self.status_updated.emit("ESP sincronizada.")
            
            # Itera sobre os pontos da trajetória e emite sinais para a Main_Window, que atualiza os plots
            for i, trajectory in enumerate(self.trajectory):
                while self.is_paused and self.is_running:
                    self.msleep(50)
                if not self.is_running:
                    break

                t, q_traj, qd_traj, qdd_traj = trajectory

                # 1) compute o comando a enviar à ESP (use a referência ou o q calculado)
                #    Para teste/eco da ESP, melhor mandar a própria referência:
                # <<< MODIFICADO: Prepara q, qd, e qdd para enviar
                q_cmd = list(np.asarray(q_traj, dtype=float).reshape(3))
                qdd_cmd = list(np.asarray(qdd_traj, dtype=float).reshape(3)) # <<< NOVO

                # 2) envie setpoint e peça medição
                # <<< MODIFICADO: Envia q, qd, e qdd
                # (V2) self.tx_json.emit({"cmd": "set_ref", "q": q_cmd})
                self.tx_json.emit({"cmd": "set_ref", "q": q_cmd, "qdd": qdd_cmd})
                self.tx_json.emit({"cmd": "get_meas"})

                # 3) aguarde CHEGAR uma medição nova
                seq_before = self._rx_seq
                
                # (V2) self.tx_json.emit({"cmd": "set_ref", "q": q_cmd})
                # self.tx_json.emit({"cmd": "set_ref", "q": q_cmd, "qdd": qdd_cmd})
                # self.tx_json.emit({"cmd": "get_meas"})

                while self.is_running and not self._wait_for_new_meas(seq_before, timeout_ms=int(max(20, 100 / self.speed_factor))):
                    self.status_updated.emit("Aguardando medição da ESP…")
                    self.tx_json.emit({"cmd": "get_meas"})
                    # seq_before permanece o mesmo; esperamos algo com _rx_seq > seq_before

                dt = self.controller.dt[i]

                if self.last_meas_qdd is not None:
                    # Por hora usa a aceleração qdd enquanto não sei calcular a aceleração angular
                
                    accel_para_integrar = np.array(self.last_meas_qdd) 

                    # Integração de Euler
                    # Vel_nova = Vel_antiga + Aceleração * tempo
                    self.qd_calculado = self.qd_calculado + (accel_para_integrar * dt)


                # 4) injete a medição no controlador
                # <<< MODIFICADO: Passa q, qd, e qdd para o controlador
                if self.controller and self.last_meas is not None:
                    try:
                        # (V2) self.controller.set_measurement(self.last_meas, t=None)
                        self.controller.set_measurement(
                            self.last_meas,       # q
                            self.qd_calculado,    # qd
                            self.last_meas_qdd,   # qdd
                            t=None                # ignora t da ESP
                        )
                    except Exception as e:
                        print(f"Erro ao chamar set_measurement: {e}") #Debug para ver onde da erro
                        pass

                # 5) roda o controle (se não houver medição, ele lança 'sem_medidoes', mas não deve ocorrer)
                try:
                    # <<< MODIFICADO: O 'e_acc' não é mais retornado pelo V3
                    # (V2) q_joints, qd_joints, qdd_joints, e_pos, e_vel, e_acc, tau = self.controller.update_control_position()
                    q_joints, qd_joints, qdd_joints, e_pos, e_vel, e_acc, tau = self.controller.update_control_position()
                except RuntimeError as ex:
                    if str(ex) == "sem_medidoes":
                        # segurança extra: volte a aguardar
                        self.status_updated.emit("Medição ainda não disponível; aguardando…")
                        continue
                    else:
                        raise

                # 6) publica para a UI
                # <<< MODIFICADO: Passa 'e_vel' no lugar do 'e_acc' (ou [0,0,0])
                # (V2) self.position_updated.emit(float(t), list(q_joints), list(qd_joints), list(qdd_joints), list(e_pos), list(e_vel), list(e_acc), list(tau))
                self.position_updated.emit(float(t), list(q_joints), list(qd_joints), list(qdd_joints), list(e_pos), list(e_vel), list(e_acc), list(tau), list(self.last_meas_i))
                
                progress = int((i + 1) / total_points * 100)
                self.progress_updated.emit(progress)
                self.status_updated.emit(f"Simulando... {progress}% | t={t:.2f}s")

                # 7) passo de tempo (a simulação avança só após medição)
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