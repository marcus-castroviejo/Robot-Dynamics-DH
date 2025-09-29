"""
=================================================================================================================
                                                Classe Robot Parameters
=================================================================================================================
- Parâmetros do robô
"""
import numpy as np
from sympy import pi, symbols, Matrix


class RobotParameters:
    """--------------------------- Dimensões físicas [m] ---------------------------"""
    L1 = 0.3        # Elo 1
    L2 = 0.5870     # Elo 3

    """--------------------------- Limites operacionais ---------------------------"""
    J1_RANGE = (-np.pi, np.pi)                              # Junta 1: R [rad]
    J2_RANGE = (np.deg2rad(90-13), np.deg2rad(90+61.5))     # Junta 2: R [rad]
    D3_MAX = 0.087                                          # Junta 3: P [m]
    
    """--------------------------- Massas [kg] ---------------------------"""
    M1 = 0.9112
    M2 = 0.0
    M3 = 1.3446
    
    """--------------------------- Centros de massa [m] ---------------------------"""
    CM1 = 0.080
    CM2 = 0.0  
    CM3 = 0.233
    
    """--------------------------- Inércias ---------------------------"""
    #     x          y          z
    I1 = [1.6209e-5, 1.5989e-5, 9.6796e-7]
    I2 = [        0,         0,         0]
    I3 = [1.2403e-4, 2.4632e-4, 1.2389e-4]
    
    """--------------------------- Tabela Denavit-Hartenberg (DH) ---------------------------"""
    q1, q2, q3 = symbols('q_1:4')
    # DH_TABLE = Matrix([
    #     # theta     d           a       alpha
    #     [ q1,       L1,         0,       pi/2],     # Junta 1
    #     [ q2,       0,          0,       pi/2],     # Junta 2
    #     [ pi/2,     q3+L2,      0,       0   ]      # Junta 3
    # ])

    # Nova versão
    DH_TABLE = Matrix([
        # theta     d           a       alpha
        [ q1+pi,    L1,         0,      -pi/2 ],    # Junta 1
        [ q2,       0,          0,      -pi/2 ],    # Junta 2
        [ 0,        q3+L2,      0,      0     ]     # Junta 3
    ])
    """--------------------------- Constantes físicas ---------------------------"""
    GRAVITY = 9.81
