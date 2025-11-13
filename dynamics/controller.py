"""
=================================================================================================================
                                                Classe RobotPlot
=================================================================================================================
- Controlador Torque Calculado
    - A trajetória desejada é configurada ao se iniciar a simulação
    - O controle de posição é realizado à cada iteração de SimulationThread.run() através de update_control_position()

Campos deste código:
["Setup Inicial"]:                      Inicialização
["Configuraçōes do Controlador"]:       Especificações, Ganhos e Trajetória
["Controle]:                            Função de update do controlador
["Métodos de Integração"]:              Euler ou Runge-Kutta4
"""
import numpy as np

class CalculatedTorqueController:
    """Controlador Torque Calculado"""

    """
    =================================================================================================================
                                                Setup inicial
    =================================================================================================================
    """
    
    """--------------------------- __init__() ---------------------------"""
    def __init__(self, robot):
        # Robô
        self.robot = robot
        self.setup_controller()
        self.set_gain_factors()

    
    """
    =================================================================================================================
                                                Configuraçōes do Controlador
    =================================================================================================================
    """

    """--------------------------- Especificações ---------------------------"""
    def setup_controller(self):
        """Especificações do sistema"""
        self.ts = 0.3                           # Settling time: 5% tolerância
        self.zeta = 0.59                        # Overshoot: 10%

        # Frequência natural e alocação de um polo p afastado
        self.wn = 3/(self.zeta*self.ts)
        self.p = 5*self.zeta*self.wn

    """--------------------------- Trajetória desejada ---------------------------"""
    def set_trajectory(self, trajectory):
        """Definindo a trajetória"""
        # Trajetória
        self.t = [point[0] for point in trajectory]
        self.dt = list(np.diff([self.t[0]] + self.t))
        self.q_traj = [point[1] for point in trajectory]
        self.qd_traj = [point[2] for point in trajectory]
        self.qdd_traj = [point[3] for point in trajectory]

        # Posição, velocidade, aceleração
        self.q_real = [self.q_traj[0]]
        self.qd_real = [[0.0, 0.0, 0.0]]
        self.qdd_real = [[0.0, 0.0, 0.0]]
        
        self.errors = []
        self.e_int = 0
        self.tau = []

        self.counter  = 0
    
    """--------------------------- Escalando os ganhos ---------------------------"""
    def set_gain_factors(self, Kp_scaling_factor=0.050, Kd_scaling_factor=0.150, Ki_scaling_factor=0.001):
        """Definindo os fatores de escala para os ganhos"""
        self.Kp_scaling_factor = Kp_scaling_factor
        self.Kd_scaling_factor = Kd_scaling_factor
        self.Ki_scaling_factor = Ki_scaling_factor
        self.calculate_gains()

        # --- estado de medição externa (ESP32) ---
        self.use_external_meas = True 
        self._meas_q = None
        self._meas_t = None
        
        # <<< MODIFICADO: Adiciona buffers para velocidade e aceleração
        self._meas_qd = None 
        self._meas_qdd = None

    # <<< MODIFICADO: A assinatura da função mudou para aceitar qd e qdd
    def set_measurement(self, q_meas, qd_meas, qdd_meas, t=None):
        """Recebe medição das juntas (ESP32)."""
        self.use_external_meas = True

        q = np.asarray(q_meas, dtype=float).reshape(3)
        qd = np.asarray(qd_meas, dtype=float).reshape(3)    # <<< MODIFICADO
        qdd = np.asarray(qdd_meas, dtype=float).reshape(3)  # <<< MODIFICADO

        # 1) filtro anti-bursts do início (evita salto inicial)
        if np.allclose(q, 0.0, atol=1e-9):
            return  # ignora pacote claramente inválido

        # 2) registra medição atual
        self._meas_q = q
        self._meas_t = float(t) if t is not None else None
        self._meas_qd = qd    # <<< MODIFICADO
        self._meas_qdd = qdd  # <<< MODIFICADO

        # 3) primeira amostra válida: alinhar estados do modelo
        if len(self.q_real) == 1 and self.counter == 0:
            # posição real inicial = medição
            self.q_real[0]  = q.copy()
            # zera velocidade e aceleração iniciais do modelo
            self.qd_real[0] = qd.copy()  # <<< MODIFICADO
            self.qdd_real[0]= qdd.copy() # <<< MODIFICADO
            # zera integral do erro
            self.e_int = np.zeros(3, dtype=float)



    
    """--------------------------- Cálculo dos ganhos ---------------------------"""
    def calculate_gains(self):
        """Ganhos Kp, Kd e Ki"""
        self.Kp = self.Kp_scaling_factor * np.diag(3*[self.wn**2 + 2*self.zeta*self.wn*self.p])     # scaling_factor: 0.050 (!!)
        self.Kd = self.Kd_scaling_factor * np.diag(3*[2*self.zeta*self.wn + self.p])                # scaling_factor: 0.150 (!!)
        self.Ki = self.Ki_scaling_factor * np.diag(3*[self.wn**2*self.p])                           # scaling_factor: 0.001 (!!)
    
    """
    =================================================================================================================
                                                Controle
    =================================================================================================================
    """

    """--------------------------- Update do controlador ---------------------------"""
    def update_control_position(self):
        """
        <<< LÓGICA V3 (MODIFICADA) >>>
        - q_real, qd_real, qdd_real = medição (ESP32)
        - O controlador calcula o 'tau' necessário com base nesses estados medidos.
        - Não há mais integração interna ou "modelo sombra".
        """
        # 1) passo temporal
        dt = self.dt[self.counter]

        # 2) referência (trajetória)
        q_traj  = np.array(self.q_traj[self.counter])
        qd_traj = np.array(self.qd_traj[self.counter])
        qdd_traj= np.array(self.qdd_traj[self.counter])

        # 3) exigir medição
        if self._meas_q is None: # <<< Assume que qd e qdd chegam junto com q
            raise RuntimeError("sem_medidoes")

        # 4) estados: Posição, Velocidade e Aceleração REAIS = medição
        #    (No V2, qd e qdd vinham de um modelo interno)
        q_real   = np.array(self._meas_q, dtype=float)
        qd_real  = np.array(self._meas_qd, dtype=float)   # <<< MODIFICADO
        qdd_real = np.array(self._meas_qdd, dtype=float)  # <<< MODIFICADO

        # 5) erros (usando estados medidos)
        e_pos = q_traj - q_real
        e_vel = qd_traj - qd_real                         # <<< MODIFICADO
        e_acc = qdd_traj - qdd_real                     # (Erro de aceleração, se necessário)
        self.e_int += e_pos * dt

        # 6) dinâmica no estado medido (q_real, qd_real)
        #    (Passamos qdd_real para manter a assinatura da função, como na V1)
        M, C, G = self.robot.calculate_dynamics(list(q_real), list(qd_real), list(qdd_real)) # <<< MODIFICADO
        M = np.array(M, dtype=np.float64)
        C = np.array(C, dtype=np.float64)
        G = np.array(G, dtype=np.float64).flatten()

        # 7) torque calculado (Computed Torque)
        #    Usa os erros medidos para calcular o torque.
        tau = M @ (qdd_traj + self.Kd @ e_vel + self.Kp @ e_pos + self.Ki @ self.e_int) + C @ qd_real + G 
        # 8) <<< MODIFICADO: LÓGICA DE INTEGRAÇÃO REMOVIDA >>>
        #    O estado "real" agora é o que vem da medição.
        #    Não precisamos mais calcular o 'qdd_model' ou integrar 'qd_new'.
        
        # =========================================================================
        # --- Início da Lógica Antiga (V1) Comentada (PARA COMPARAÇÃO) ---
        # =========================================================================
        # # Na V1, os estados reais vinham do loop anterior
        # q_real_v1_anterior = np.array(self.q_real[-1])
        # qd_real_v1_anterior = np.array(self.qd_real[-1])
        #
        # # Os erros eram calculados com base nesses estados internos
        # error_v1 = q_traj - q_real_v1_anterior
        # e_dot_v1 = qd_traj - qd_real_v1_anterior
        # self.e_int += error_v1 * dt
        #
        # # O torque era calculado
        # tau_v1 = M @ (qdd_traj + self.Kd @ e_dot_v1 + self.Kp @ error_v1 + self.Ki @ self.e_int) + C @ qd_real_v1_anterior + G
        #
        # # A aceleração real era DESCOBERTA a partir do torque
        # qdd_real_v1 = np.linalg.solve(M, tau_v1 - C @ qd_real_v1_anterior - G)
        #
        # # E então, a nova posição e velocidade eram CALCULADAS por integração
        # q_real_v1, qd_real_v1 = self.integrate_runge_kutta4(q_real_v1_anterior, qd_real_v1_anterior, qdd_real_v1, dt)
        #
        # # E salvas
        # self.qdd_real.append(qdd_real_v1)
        # self.qd_real.append(qd_real_v1)
        # self.q_real.append(q_real_v1)
        # =========================================================================
        # --- Fim da Lógica Antiga (V1) Comentada ---
        # =========================================================================


        # 9) salvar histórico (agora salvando os dados MEDIDOS)
        self.errors.append(e_pos)
        self.tau.append(tau)
        self.qdd_real.append(qdd_real)  # <<< MODIFICADO
        self.qd_real.append(qd_real)    # <<< MODIFICADO
        self.q_real.append(q_real)

        # 10) avança
        self.counter += 1

        # <<< MODIFICADO: Retorna os estados reais (medidos)
        return (q_real, qd_real, qdd_real, e_pos, e_vel, e_acc, tau) # removi e_acc

    """
    =================================================================================================================
                                                Métodos de Integração
    =================================================================================================================
    """

    # (Estes métodos não são mais usados pelo 'update_control_position', 
    # mas podem ser úteis para outras partes da simulação, então é bom manter)

    """--------------------------- Integração: Euler ---------------------------"""
    @staticmethod
    def integrate_euler(q, qd, qdd, dt):
        qd_new = qd + qdd * dt
        q_new = q + qd_new * dt

        return q_new, qd_new

    """--------------------------- Integração: Runge-Kutta 4 ---------------------------"""
    @staticmethod
    def integrate_runge_kutta4(q, qd, qdd, dt):
        # k1
        k1_q = qd
        k1_qd = qdd
        
        # k2
        k2_q = qd + 0.5 * dt * k1_qd
        k2_qd = qdd  # qdd assumido constante no intervalo
        
        # k3
        k3_q = qd + 0.5 * dt * k2_qd
        k3_qd = qdd
        
        # k4
        k4_q = qd + dt * k3_qd
        k4_qd = qdd
        
        q_new = q + (dt/6) * (k1_q + 2*k2_q + 2*k3_q + k4_q)
        qd_new = qd + (dt/6) * (k1_qd + 2*k2_qd + 2*k3_qd + k4_qd)
        
        return q_new, qd_new