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
        self.calculate_gains()
    
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

    """--------------------------- Cálculo dos ganhos ---------------------------"""
    def calculate_gains(self):
        """Ganhos Kp, Kd e Ki"""
        self.Kp = 0.05 * np.diag(3*[self.wn**2 + 2*self.zeta*self.wn*self.p])   # scaling_factor: 0.050 (!!)
        self.Kd = 0.15 * np.diag(3*[2*self.zeta*self.wn + self.p])              # scaling_factor: 0.150 (!!)
        self.Ki = 0.001 * np.diag(3*[self.wn**2*self.p])                        # scaling_factor: 0.001 (!!)
    
    """--------------------------- Trajetória desejada ---------------------------"""
    def set_trajectory(self, trajectory):
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
        
        # Erros, Tau
        self.errors = []
        self.e_int = 0
        self.tau = []

        # Counter
        self.counter = 0
    
    """
    =================================================================================================================
                                                Controle
    =================================================================================================================
    """

    """--------------------------- Update do controlador ---------------------------"""
    def update_control_position(self):
        """Controle da posição do robô (sistema de coordenada das juntas)"""
        # Delta de tempo que atualiza a cada iteração
        dt = self.dt[self.counter]

        # Posições desejadas (trajetória): _traj
        q_traj = np.array(self.q_traj[self.counter])
        qd_traj = np.array(self.qd_traj[self.counter])
        qdd_traj = np.array(self.qdd_traj[self.counter])

        # Posições reais: _real
        q_real = np.array(self.q_real[-1])
        qd_real = np.array(self.qd_real[-1])
        qdd_real = np.array(self.qdd_real[-1])

        # Erros
        error = q_traj - q_real
        e_dot = qd_traj - qd_real
        self.e_int += error * dt

        # Matriz de Inércia, Matriz de Coriolis e Vetor de Gravidade
        M, C, G = self.robot.calculate_dynamics(list(q_real), list(qd_real), list(qdd_real))
        M = np.array(M, dtype=np.float64)
        C = np.array(C, dtype=np.float64)
        G = np.array(G, dtype=np.float64).flatten()

        # Torque Calculado:
        tau = M @ (qdd_traj + self.Kd @ e_dot + self.Kp @ error + self.Ki @ self.e_int) + C @ qd_real + G
        qdd_real = np.linalg.solve(M, tau - C @ qd_real - G)

        # Integração numérica
        # q_real, qd_real = self.integrate_euler(q_real, qd_real, qdd_real, dt)             # Euler
        q_real, qd_real = self.integrate_runge_kutta4(q_real, qd_real, qdd_real, dt)        # Runge-Kutta 4

        # Prints para Debugging...
        """print("\n--> ", self.counter, "\n")
        print("M:", list(M))
        print("C:", list(C))
        print("G:", list(G))
        print("tau:", tau)
        print("q:", q_real)"""

        # Salva os dados
        self.errors.append(error)
        self.tau.append(tau)
        self.qdd_real.append(qdd_real)
        self.qd_real.append(qd_real)
        self.q_real.append(q_real)

        # Atualiza o Counter
        self.counter += 1

        return (error, q_real, qd_real, qdd_real, tau)

    """
    =================================================================================================================
                                                Métodos de Integração
    =================================================================================================================
    """

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
