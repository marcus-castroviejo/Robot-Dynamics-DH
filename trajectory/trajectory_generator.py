"""
============================================
    Classe TrajectoryGeneratorRRP
============================================
"""
import numpy as np


class TrajectoryGeneratorRRP:
    """Gerador de trajetórias"""
    
    @staticmethod
    def coeff_traj(q0, qf, tf):
        """Calcula coeficientes do polinômio de quinta ordem"""
        D = qf - q0
        a0 = q0
        a1 = 0
        a2 = 0
        a3 = 10 * D / tf**3
        a4 = -15 * D / tf**4
        a5 = 6 * D / tf**5
        return [a0, a1, a2, a3, a4, a5]
    
    @staticmethod
    def calc_traj(a, t):
        """Calcula trajetória, velocidade e aceleração"""
        q = a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3 + a[4]*t**4 + a[5]*t**5
        qd = a[1] + 2*a[2]*t + 3*a[3]*t**2 + 4*a[4]*t**3 + 5*a[5]*t**4
        qdd = 2*a[2] + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3
        return q, qd, qdd
    
    @classmethod
    def generate_trajectory(cls, q0, qf, tf, dt=0.01):
        """Gera a trajetória completa"""
        t_grid = np.arange(0, tf + dt, dt)
        
        # Calcular coeficientes para cada junta
        coeffs = [cls.coeff_traj(q0[i], qf[i], tf) for i in range(3)]
        
        # Gerar trajetórias
        trajectory = []
        for t in t_grid:
            q_t = [cls.calc_traj(coeffs[i], t)[0] for i in range(3)]
            qd_t = [cls.calc_traj(coeffs[i], t)[1] for i in range(3)]
            qdd_t = [cls.calc_traj(coeffs[i], t)[2] for i in range(3)]
            trajectory.append((t, q_t, qd_t, qdd_t))

        return trajectory
