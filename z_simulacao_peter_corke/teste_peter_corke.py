# Código feito para comparar com os resultados da implementação própria e possibilitar simulação.
# Arquivos .py para que os gráficos e simulações sejam gerados corretamente.

from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
from roboticstoolbox.tools.trajectory import mtraj, quintic
import matplotlib.pyplot as plt
import numpy as np
import sympy as sp

# DEFINIÇÃO DO ROBÔ ============================================================
# Dimensões e limites
L = [.3, .5870, 0]
qlim = [
    [-np.pi, np.pi],
    [np.deg2rad(90-13), np.deg2rad(90+61.5)],
    [0, .087]
]
offset = [0, 0, L[1]]  # Obs: d3 = L2 + q3
# Massas, centro de massas e inercias
m = [0.9112, 0, 1.3446]
cm = [
    [0, 0, 0.08],
    [0, 0, 0],
    [0, 0, 0.233]
]
I = [
    [1.6209e-5, 1.5989e-5, 9.6796e-7, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [1.2403e-4, 2.4632e-4, 1.2389e-4, 0, 0, 0],
]
# Coeficientes de atrito viscoso e estático (Coulomb)
B = [0.001, 0.001, 0.0005]
Tc = [
    [0.02, -0.02],
    [0.015, -0.015],
    [0.01, -0.01]
]
# Denavit Hartenberg
a = [0, 0, 0]
d = [L[0], 0, 0]
alpha = [np.pi/2, np.pi/2, 0]
theta = [0, 0, np.pi/2]

# Elos (RRP)
link1 = RevoluteDH(
    d=d[0], a=a[0], alpha=alpha[0],offset=offset[0],
    qlim=qlim[0],m=m[0],r=cm[0],I=I[0],B=B[0],Tc=Tc[0],name='q1'
)
link2 = RevoluteDH(
    d=d[1], a=a[1], alpha=alpha[1],offset=offset[1],
    qlim=qlim[1],m=m[1],r=cm[1],I=I[1],B=B[1],Tc=Tc[1],name='q2'
)
link3 = PrismaticDH(
    theta=theta[2], a=a[2], alpha=alpha[2], offset=offset[2],
    qlim=qlim[2],m=m[2],r=cm[2],I=I[2],B=B[2],Tc=Tc[2],name='q3'
)
# Criando o robô
cocoabot = DHRobot([link1, link2, link3], name='Cocoabot')
print(cocoabot)


# TRAJETÓRIA ============================================================
# Condicoes de contorno
q0 = np.array([0, np.pi/2, 0])
qf = np.array([np.pi/2, (3/4)*np.pi, 0.08])
# Parâmetros
tf = 5.0 
t = np.linspace(0, tf, 300)
# Trajetória Polinômio de 5 grau
traj = mtraj(quintic, q0, qf, t)
q = traj.q
qd = traj.qd
qdd = traj.qdd


# CINEMÁTICA E DINÂMICA ============================================================
# Cinemática direta
T = cocoabot.fkine(q)
# Cinemática inversa
q_cin = cocoabot.ikine_LM(T)
# Jacobianos
J = cocoabot.jacob0(q[0])       # da base
Jee = cocoabot.jacobe(q[0])     # do end-effector
sp.Matrix(J)
# Dinâmica
tau_din = cocoabot.rne(q, qd, qdd)          # Dinâmica inversa
qdd_din = cocoabot.accel(q, qd, tau_din)    # Dinâmica direta
G = cocoabot.gravload(q)             # G(q)
C = cocoabot.coriolis(q, qd)         # C(q, qd)
M = cocoabot.inertia(q)              # M(q)


# PLOTS ============================================================
traj.plot()
cocoabot.plot(q)
cocoabot.teach(q=q0)


# ANIMAÇÃO DA TRAJETÓRIA ============================================================
# Salvando frames para animação
# import os
# import matplotlib.pyplot as plt

# output_dir = "frames"
# os.makedirs(output_dir, exist_ok=True)

# for i, qi in enumerate(q):
#     cocoabot.plot(qi, block=False)
#     plt.savefig(f"{output_dir}/frame_{i:03d}.png")
#     plt.close()
# No Terminal: ffmpeg -framerate 20 -i frames/frame_%03d.png -c:v libx264 -pix_fmt yuv420p simulacao.mp4


plt.show()

