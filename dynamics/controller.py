"""
=================================================================================================================
                                                Classe RobotPlot
=================================================================================================================
- Controlador Torque Calculado
    - A trajetória desejada é configurada ao se iniciar a simulação
    - O controle de posição é realizado à cada iteração de SimulationThread.run() através de update_control_position()

Características:
- Recebe medições de posição da ESP32 (via CommunicationManager)
- Calcula velocidade e aceleração por diferenciação numérica
- Lei de controle: τ = M(q̈_d + Kd·ė + Kp·e + Ki·∫e) + C·q̇ + G
- Comando de posição via mola virtual: q_cmd = q + Kt·τ

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
        """Inicializa o controlador"""
        # Robô
        self.robot = robot
        self.setup_controller()
        self.set_gain_factors()

        # Usado para fazer a diferenciação númerica.
        self.last_q_real = np.array([0.0, 0.0, 0.0])
        self.last_qd_real = np.array([0.0, 0.0, 0.0])
    
    """
    =================================================================================================================
                                                Configuraçōes do Controlador
    =================================================================================================================
    """

    """--------------------------- Especificações ---------------------------"""
    def setup_controller(self):
        """Especificações do sistema de controle"""
        self.ts = 0.3                           # Settling time: 5% tolerância
        self.zeta = 0.59                        # Overshoot: 10%

        # Frequência natural e alocação de um polo p afastado
        self.wn = 3/(self.zeta*self.ts)
        self.p = 5*self.zeta*self.wn

    """--------------------------- Trajetória desejada ---------------------------"""
    def set_trajectory(self, trajectory):
        """Definindo a trajetória desejada"""
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
    
    """--------------------------- Cálculo dos ganhos ---------------------------"""
    def calculate_gains(self):
        """Ganhos Kp, Kd e Ki"""
        self.Kp = self.Kp_scaling_factor * np.diag(3*[self.wn**2 + 2*self.zeta*self.wn*self.p])     # scaling_factor: 0.050 (!!)
        self.Kd = self.Kd_scaling_factor * np.diag(3*[2*self.zeta*self.wn + self.p])                # scaling_factor: 0.150 (!!)
        self.Ki = self.Ki_scaling_factor * np.diag(3*[self.wn**2*self.p])

        # self.Kt_ganho = 1
        self.Kt = self.Kt_ganho * np.identity(3)

    """--------------------------- Escalando os ganhos ---------------------------"""
    def set_gain_factors(self, Kp_scaling_factor=0.050, Kd_scaling_factor=0.150, Ki_scaling_factor=0.001, Kt=1):
        """Definindo os fatores de escala para os ganhos"""
        self.Kp_scaling_factor = Kp_scaling_factor
        self.Kd_scaling_factor = Kd_scaling_factor
        self.Ki_scaling_factor = Ki_scaling_factor
        self.Kt_ganho = Kt
        self.calculate_gains()

        # --- estado de medição externa (ESP32) ---
        self.use_external_meas = True 
        self._meas_q = None
        self._meas_t = None
        
    """--------------------------- Estados reais das juntas ---------------------------"""
    def set_measurement(self, q_meas, t=None):
        """Recebe medição de posição das juntas (da ESP32 via CommunicationManager)"""
        self.use_external_meas = True

        q = np.asarray(q_meas, dtype=float).reshape(3)

        # 1) filtro anti-bursts do início (evita salto inicial)
        if np.allclose(q, 0.0, atol=1e-9):
            return  # ignora pacote claramente inválido

        # 2) registra medição atual
        self._meas_q = q
        self._meas_t = float(t) if t is not None else None

        # 3) primeira amostra válida: alinhar estados do modelo
        if len(self.q_real) == 1 and self.counter == 0:
            # Define Posição real inicial = medição
            self.q_real[0]  = q.copy()
            # Zera velocidade e aceleração iniciais do modelo
            self.qd_real[0] = np.zeros(3, dtype=float)
            self.qdd_real[0]= np.zeros(3, dtype=float)
            # Zera integral do erro
            self.e_int = np.zeros(3, dtype=float)
            # Estado inicial para a diferenciação numérica
            self.last_q_real = q.copy()
            self.last_qd_real = np.zeros(3, dtype=float)
    
    
    """
    =================================================================================================================
                                                Controle
    =================================================================================================================
    """

    """--------------------------- Update do controlador ---------------------------"""
    def update_control_position(self):
        """
        Atualiza a lei de controle

        Fluxo:
        1. Obtém trajetória desejada (q_d, qd_d, qdd_d)
        2. Obtém posição medida (q) da ESP32
        3. Calcula velocidade (qd) e aceleração (qdd) por diferenciação
        4. Calcula erros (e, ed, edd)
        5. Calcula dinâmica (M, C, G) no estado real
        6. Calcula torque: τ = M(qdd_d + PID) + C·qd + G
        7. Calcula comando via mola virtual: q_cmd = q + Kt·τ
        
        Returns:
            tuple: (q_command, q_real, qd_real, qdd_real, e_pos, e_vel, e_acc, tau)
        """
        # 1) passo temporal
        dt = self.dt[self.counter]
        if dt <= 1e-9:                      # Como é diferenciação isso evita dividir por zero.
            dt = 1e-9

        # 2) Referência (trajetória desejada)
        q_traj  = np.array(self.q_traj[self.counter])
        qd_traj = np.array(self.qd_traj[self.counter])
        qdd_traj= np.array(self.qdd_traj[self.counter])

        # 3) Exige medição disponível
        if self._meas_q is None:
            raise RuntimeError("sem_medidoes")

        # 4) Estados Reais: Posição, Velocidade e Aceleração [medição da ESP32]
        q_real   = np.array(self._meas_q, dtype=float)

        # 5) Diferenciação numérica: cálculo da velocidade e aceleração
        qd_real = (q_real - self.last_q_real) / dt
        qdd_real = (qd_real - self.last_qd_real) / dt

        # 6) Erros (usando estados medidos e calculados)
        e_pos = q_traj - q_real
        e_vel = qd_traj - qd_real
        e_acc = qdd_traj - qdd_real
        self.e_int += e_pos * dt

        # 7) Dinâmica no estado medido (q_real, qd_real, qdd_real)
        M, C, G = self.robot.calculate_dynamics(list(q_real), list(qd_real), list(qdd_real))
        M = np.array(M, dtype=np.float64)
        C = np.array(C, dtype=np.float64)
        G = np.array(G, dtype=np.float64).flatten()

        # 8) Torque Calculado (Computed Torque)
        # Lei de controle: τ = M(q̈_d + Kd·ė + Kp·e + Ki·∫e) + C·q̇ + G
        tau = M @ (qdd_traj + self.Kd @ e_vel + self.Kp @ e_pos + self.Ki @ self.e_int) + C @ qd_real + G 
        
        # 9) Comando de Posição via Mola Virtual
        # Converte torque em deslocamento de posição
        q_command = q_real + self.Kt @ tau

        # 10) salva valores para a próxima derivada
        self.last_q_real = q_real.copy()
        self.last_qd_real = qd_real.copy()

        # 11) Salvar histórico (dados medidos e calculados)
        self.errors.append(e_pos)
        self.tau.append(tau)
        self.qdd_real.append(qdd_real)
        self.qd_real.append(qd_real)
        self.q_real.append(q_real)

        # 12) Avança contador de iterações
        self.counter += 1

        return (q_command, q_real, qd_real, qdd_real, e_pos, e_vel, e_acc, tau)

    """
    =================================================================================================================
                                    Métodos de Integração (não são mais utilizados)
    =================================================================================================================
    """

    # Estes métodos não são mais usados pelo 'update_control_position', 
    # mas podem ser úteis para outras partes da simulação, então é bom manter

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