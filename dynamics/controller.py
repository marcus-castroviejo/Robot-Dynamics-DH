"""
=================================================================================================================
                                                Classe RobotPlot
=================================================================================================================
"""
import numpy as np

class CalculatedTorqueController:
    """Controlador Torque Calculado"""

    """--------------------------- __init__() ---------------------------"""
    def __init__(self, robot):
        # Robô
        self.robot = robot
        self.setup_controller()
        self.calculate_gains()
        
        # Erros
        self.errors = []

        # Posição, velocidade, aceleração
        self.q_real = [[0.0, 0.0, 0.0]]
        self.qd_real = [[0.0, 0.0, 0.0]]
        self.qdd_real = [[0.0, 0.0, 0.0]]

        # Torque e Força
        self.tau = []
    
    """--------------------------- Especificações do Controlador ---------------------------"""
    def setup_controller(self):
        """Especificações do sistema"""
        self.ts = 0.3                # Settling time: 5% tolerância
        self.zeta = 0.59             # Overshoot: 10%

        # Frequência natural e alocação de um polo p afastado
        self.wn = 3/(self.zeta*self.ts)
        self.p = 5*self.zeta*self.wn

    """--------------------------- Cálculo dos Ganhos ---------------------------"""
    def calculate_gains(self):
        """Ganhos Kp, Kd e Ki"""
        self.Kp = np.diag(3*[self.wn**2 + 2*self.zeta*self.wn*self.p])
        self.Kd = np.diag(3*[2*self.zeta*self.wn + self.p])
        self.Ki = np.diag(3*[self.wn**2*self.p])

    """--------------------------- Controla a posição ---------------------------"""
    def control_position(self, q_traj, qd_traj, qdd_traj):
        """Controle da posição do robô (sistema de coordenada das juntas)"""
        dt = 50.0/1000.0                                                            # (!!!)
        # Posições desejadas (trajetória): _traj
        q_traj = np.array(q_traj)
        qd_traj = np.array(qd_traj)
        qdd_traj = np.array(qdd_traj)

        # Posições reais: _real
        q_real = np.array(self.q_real[-1])
        qd_real = np.array(self.qd_real[-1])

        # Erros
        error = q_traj - q_real
        e_dot = qd_traj - qd_real
        e_int = error * dt

        # Matriz de Inércia, Matriz de Coriolis e Vetor de Gravidade
        M, C, G = self.robot.calculate_dynamics(list(q_traj), list(qd_traj), list(qdd_traj))
        M = np.array(M, dtype=np.float64)
        C = np.array(C, dtype=np.float64)
        G = np.array(G, dtype=np.float64).flatten()                             # PROBLEMA (!!!) [nan]
        
        # Torque Calculado:
        tau = M @ (qdd_traj + self.Kd @ e_dot +self.Kp @ error + self.Ki @ e_int) + C @ qd_real + G
        qdd_real = np.linalg.solve(M, tau - C @ qd_real - G)
        qd_real = qd_real + qdd_real * dt
        q_real = q_real + qd_real * dt

        # Save the data
        self.errors.append(error)
        self.tau.append(tau)
        self.qdd_real.append(qdd_real)
        self.qd_real.append(qd_real)
        self.q_real.append(q_real)

        return (error, q_real, qd_real, qdd_real, tau)
