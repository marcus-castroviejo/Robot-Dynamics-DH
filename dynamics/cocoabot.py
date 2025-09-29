"""
=================================================================================================================
                                                Classe Cocoabot 
=================================================================================================================
- Dinâmica e Cinemática do CocoaBot

Campos deste código:
["Setup Inicial"]:              Inicialização, Configuração, Criação do CocoaBot
["Dinâmica e Cinemática"]:      Forward Kynematics, Dinâmica [Inércia, Coriolis, Gravidade]
"""
from sympy import *
import numpy as np
from dynamics import Robot
from config import RobotParameters as params


class CocoaBot:
    """Adaptação da classe Robot (roboot_dynamics.py) para o Cocoabot (RRP)"""
    
    """
    =================================================================================================================
                                                Setup inicial
    =================================================================================================================
    """

    """--------------------------- __init__() ---------------------------"""
    def __init__(self):
        # Parâmetros do robô RRP
        self.setup_robot()
        self.setup_full_robot()
    
    """--------------------------- Parâmetros Numéricos ---------------------------"""
    def setup_robot(self):
        """Configuração do robô: parâmetros"""
        # Parâmetros físicos
        self.L1, self.L2 = params.L1, params.L2
        self.J1_range = params.J1_RANGE
        self.J2_range = params.J2_RANGE
        self.d3_max = params.D3_MAX
        # Massas, Inércias, Centros de massa e Gravidade
        self.masses = Matrix([params.M1, params.M2, params.M3])
        self.inertias = [diag(*params.I1), diag(*params.I2), diag(*params.I3)]
        self.r_cis = [Matrix([0, 0, cm]) for cm in [params.CM1, params.CM2, params.CM3]]
        self.g_vec = Matrix([0, 0, -params.GRAVITY])
        # Tabela DH
        self.dh_table = params.DH_TABLE
    
    """--------------------------- Criação do CocoaBot ---------------------------"""
    def setup_full_robot(self):
        """Cria o robo da classe Robot"""
        self.robot_dynamics = Robot(
            configuration='RRP',
            dh_table=self.dh_table,
            masses=self.masses,
            r_cis=self.r_cis,
            inertias=self.inertias,
            g_vec=self.g_vec
        )
    
    """
    =================================================================================================================
                                                Dinâmica e Cinemática
    =================================================================================================================
    """

    """--------------------------- Forward Kinematics: posição do efetuador final ---------------------------"""
    def forward_kinematics(self, q1, q2, d3):
        """Cinemática direta"""
        r = self.robot_dynamics
        q = [q1, q2, d3]
        pos = r.eval_matrix(matrix=r.base_to_end_effector[:3,3], q=q)
        return np.array(pos, dtype=float).flatten()

    """--------------------------- Forward Kinematics: juntas -> cartesiano ---------------------------"""
    def get_joint_positions(self, q1, q2, d3):
        """Posições das juntas (desejadas)"""
        r = self.robot_dynamics
        q = [q1, q2, d3]
        positions = [np.array([0.0, 0.0, 0.0])]        
        joint_positions = [r.eval_matrix(matrix=b2j[:3,3], q=q) for b2j in r.base_to_joint]
        positions.extend([np.array(posi, dtype=float).flatten() for posi in joint_positions])
        return positions
    
    """--------------------------- Inercia, Coriolis e Gravidade ---------------------------"""
    def calculate_dynamics(self, q, qd, qdd):
        """Calcular dinâmica"""
        # Use sua classe Robot completa
        r = self.robot_dynamics
        M = r.eval_matrix(r.inertia_matrix, q=q)
        C = r.eval_matrix(r.coriolis_matrix, q=q, dq=qd)
        G = r.eval_matrix(r.gravity_vector, q=q)
        return M, C, G


    """--------------------------- Inverse Kinematics: cartesiano -> juntas ---------------------------"""
    def inverse_kinematics(self, target_x, target_y, target_z):
        """Cinemática inversa"""
        try:
            # q1: orientação no plano XY
            q1 = np.arctan2(target_y, target_x)
            
            # Alcance radial
            r = np.sqrt(target_x**2 + target_y**2)
            
            # Altura relativa
            z_rel = self.L1 - target_z
            
            # q2: orientação no plano RZ
            q2 = np.arctan2(r, z_rel)

            # Comprimento do Elo 2
            hypotenuse = np.sqrt(r**2 + z_rel**2)
            
            # d3: extensão prismática
            d3 = hypotenuse - self.L2
            
            return [q1, q2, d3]
            
        except Exception as e:
            return None