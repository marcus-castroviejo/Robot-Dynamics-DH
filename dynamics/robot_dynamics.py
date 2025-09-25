"""
=================================================================================================================
                                                Classe Robot
=================================================================================================================
"""
from sympy import *
import pandas as pd
import numpy as np


class Robot:
    def __init__(self, configuration:str, dh_table:Matrix, name:str='Cocoabot', **matrices):
        """
        Initializa o robô com a configuração, tabela DH e matrizes necessárias.   
        """
        self.name = name
        self.joint_types = list(configuration)
        self.dof = len(self.joint_types)

        self.joints = pd.DataFrame(columns=['type', 'theta', 'd', 'a', 'alpha'])
        self.t = symbols('t')
        self.q = [Function(f'q{i+1}')(self.t) for i in range(self.dof)]
        self.dq = [qi.diff(self.t) for qi in self.q]
        self.ddq = [dqi.diff(self.t) for dqi in self.dq]
        self.set_matrices(**matrices)
        self.set_dh_params(dh_table)
        self.get_joints()
        self.compute()

    def set_matrices(self, **matrices):
        """
        Estabelece as matrizes necessárias para a dinâmica do robô.
        Se não forem fornecidas, usa símbolos simbólicos para as matrizes.
        Matrizes:
        - masses:     Matriz com as massas dos links: [m1, m2, m3].T (nx1)
        - r_cis:      Matriz com o centro de massa de cada link (local): [ci1, ci2, ci3].T (nx1)
        - inertias:   Matriz com a inércia de cada link: [[I11:4], [I21:4], [I31:4]] n*(3x3)
        - g_vec:      Matriz com o vetor de gravidade: [0 0 -g].T (3x1)
        """
        self.masses_symbols = Matrix(symbols(f'm1:{self.dof+1}'))
        self.masses = matrices.get('masses', self.masses_symbols)
        self.r_cis_local = matrices.get('r_cis', [Matrix(symbols(f'r_{i+1}x:z')) for i in range(self.dof)])
        self.inertias_symbols = [symbols(f'I{i+1}_1:4_1:4') for i in range(3)]
        self.inertias = matrices.get('inertias', [Matrix(3,3,self.inertias_symbols[i]) for i in range(self.dof)])
        self.g_symbol = symbols('g')
        self.g_vec = matrices.get('g_vec', Matrix([0, 0, -self.g_symbol]))
    
    def set_dh_params(self, dh_table):
        """
        Estabelece os parâmetros DH do robô.
        """
        self.thetas = dh_table[:,0]
        self.ds = dh_table[:,1]
        self._as_ = dh_table[:,2]
        self.alphas = dh_table[:,3]
    
    def get_joints(self):
        """
        Converte os parâmetros DH em um DataFrame com as informações das juntas.
        Transforma os parâmetros simbólicos q em funções de tempo q(t).
        """
        q = symbols(f'q_1:{self.dof+1}')
        for tp in self.joint_types:
            if tp == 'R': self.thetas = self.thetas.subs(zip(q,self.q))
            if tp == 'P': self.ds = self.ds.subs(zip(q,self.q))
        self.joints = pd.DataFrame(list(zip(self.joint_types, Matrix(self.thetas), self.ds, self._as_, self.alphas)), 
                                   columns=['type', 'theta', 'd', 'a', 'alpha'], 
                                   index=np.arange(self.dof)+1)
    
    def compute(self):
        """
        Calcula todas as matrizes necessárias para a dinâmica do robô.
        """
        # Kynematics
        self.dh_matrices = [self.compute_dh_matrix(joint_index=i+1) for i in range(self.dof)]
        self.base_to_joint = [self.calculate_base_to_joint(joint_end=i+1) for i in range(self.dof)]
        self.base_to_end_effector = self.base_to_joint[-1]
        self.jacobian = self.calculate_jacobian()
        # Center of Mass
        self.r_cis_global = [self.calculate_r_i_ci(joint_index=i+1) for i in range(self.dof)]
        self.jacobian_ci = self.calculate_jacobian(com=True)
        # Dynamics
        self.inertia_matrix = self.calculate_inertia_matrix()
        self.coriolis_matrix = self.calculate_coriolis_matrix()
        self.gravity_vector = self.calculate_gravity_vector()
        self.tau = self.compute_inverse_dynamics()

    def compute_dh_matrix(self, joint_index:int):
        """
        Calcula a matriz de transformação DH para a junta especificada.
        """
        assert joint_index > 0, "Must be positive."
        i = joint_index - 1
        theta, d, a, alpha = self.thetas[i], self.ds[i], self._as_[i], self.alphas[i]
        T = Matrix([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                     [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                     [0, sin(alpha), cos(alpha), d],
                     [0, 0, 0, 1]])
        return T
    
    def calculate_joint_to_joint(self, joint_start:int, joint_end:int):
        """
        Calcula a matriz de transformação de uma junta para outra.
        """
        assert joint_start > 0, "Must be positive."
        assert joint_end >= joint_start, "End must be greater than start."
        result = eye(self.dof + 1)
        for i in range(joint_start, joint_end+1):
            result = result @ self.dh_matrices[i-1]
        return simplify(result)
    
    def calculate_base_to_joint(self, joint_end):
        """
        Calcula a matriz de transformação da base para a junta especificada.
        """
        return self.calculate_joint_to_joint(joint_start=1, joint_end=joint_end)
    
    def calculate_base_to_end_effector(self):
        """
        Calcula a matriz de transformação da base para o end-effector.
        """
        return self.calculate_base_to_joint(joint_end=self.dof)
    
    def get_rotation(self, matrix):
        """
        Separa a matriz de rotação do restante da matriz de transformação.
        """
        return matrix[:self.dof,:self.dof]
    
    def get_translation(self, matrix):
        """
        Separa o vetor de translação do restante da matriz de transformação.
        """
        return matrix[:self.dof,-1:]

    def z_i_minus_one(self, joint_index):
        """
        Calcula o vetor z_i-1, que é o eixo de rotação da junta i-1, usado para calcular o Jacobiano.
        """
        result = eye(self.dof)
        for i in range(1, joint_index):
            result = result @ self.get_rotation(self.dh_matrices[i-1])
        # result = self.get_rotation(self.base_to_joint[joint_index-1])
        result = result @ Matrix([0, 0, 1])
        return simplify(result)
    
    def r_i_minus_one_to_n(self, joint_index, joint_ci=None):
        """
        Calcula a posição do link relativo ao end-effector.
        """
        # com = joint_ci is not None
        if not joint_ci:
            return self.get_translation(self.base_to_end_effector - self.base_to_joint[joint_index-1]) \
                    if joint_index > 1 else self.get_translation(self.base_to_end_effector)
        else:
            return self.r_cis_global[joint_ci-1] - self.get_translation(self.base_to_joint[joint_index - 1])

    def calculate_r_i_ci(self, joint_index):
        """
        Calcula a posição do centro de massa do link relativo ao end-effector.
        """
        r_local = Matrix.vstack(self.r_cis_local[joint_index-1], Matrix([1]))
        r_ci_global = self.base_to_joint[joint_index-1] @ r_local
        return simplify(r_ci_global[:-1,:])

    def calculate_jacobian_col_i(self, joint_index:int, joint_ci=None):
        """
        Calcula a coluna i da matriz jacobiana J(q) para o joint_index.
        """
        # com = joint_ci is not None
        if joint_ci:
            if joint_ci < joint_index:
                return Matrix(6*[0])
        joint_type = self.joint_types[joint_index-1]
        if joint_type == 'R':
            J_vi = Matrix(np.cross(self.z_i_minus_one(joint_index).T, self.r_i_minus_one_to_n(joint_index, joint_ci).T)).T
            J_wi = self.z_i_minus_one(joint_index)
        elif joint_type == 'P':
            J_vi = self.z_i_minus_one(joint_index)
            J_wi = Matrix([0, 0, 0])
        J_i = Matrix(np.vstack((J_vi, J_wi)))
        return simplify(J_i)

    def calculate_jacobian(self, com=False):
        """
        Calcula a matriz jacobiana J(q). [6 x n]
        - Se com=True, calcula a jacobiana considerando o centro de massa de cada link.
        """
        if not com:
            return Matrix(np.hstack(tuple((self.calculate_jacobian_col_i(i+1) for i in range(self.dof)))))
        else:
            return [Matrix(np.hstack(tuple((self.calculate_jacobian_col_i(j+1, i+1) 
                    for j in range(self.dof)))))
                    for i in range(self.dof)]

    def calculate_inertia_matrix(self):
        """
        Calcula a matriz de inércia M(q). [n x n]
        """
        inertia_matrix = zeros(self.dof, self.dof)

        for i in range(self.dof):
            jv = self.jacobian_ci[i][:3, :]
            jw = self.jacobian_ci[i][3:, :]
            inertia_matrix += self.masses[i] * (jv.T @ jv) + (jw.T @ self.inertias[i] @ jw)

        return simplify(inertia_matrix)

    def calculate_coriolis_matrix(self):
        """
        Calcula a matriz de Coriolis C(q, dq). [n x n]
        """
        n = self.dof
        M = self.inertia_matrix
        C = zeros(n, n)
        for i in range(n):
            for j in range(n):
                for k in range(n):
                    c_ijk = 0.5 * (diff(M[i, k], self.q[j]) + diff(M[i, j], self.q[k]) - diff(M[j, k], self.q[i]))
                    C[i, j] += c_ijk * self.dq[k]

        return simplify(C)

    def calculate_gravity_vector(self):
        """
        Calcula o vetor de gravidade G(q). [n x 1]
        """
        U = 0
        for i, com in enumerate(self.r_cis_global):
            U += self.masses[i] * self.g_vec.dot(com)
        # Differentiating the total potential energy
        G = Matrix([diff(U, q_i) for q_i in self.q])
        return G
    
    def compute_inverse_dynamics(self):
        """
        Cálculo dos tau necessários para a dinâmica inversa do robô. [n x 1]
        """
        self.q_d = [Function(f'q_{i+1}d')(self.t) for i in range(self.dof)]
        self.dq_d = [qi.diff(self.t) for qi in self.q_d]
        self.ddq_d = [dqi.diff(self.t) for dqi in self.dq_d]

        M = self.inertia_matrix
        C = self.coriolis_matrix
        G = self.gravity_vector
        tau = M.subs(zip(self.q,self.q_d)) @ Matrix(self.ddq_d) + \
              C.subs(zip(self.dq, self.dq_d)).subs(zip(self.q,self.q_d)) @ Matrix(self.dq_d) + \
              G.subs(zip(self.q,self.q_d))
        return simplify(tau)

    def eval_dinamics(self, tau, q_d=None, dq_d=None, ddq_d=None):
        """
        Substituição dos valores das variáveis q, dq e ddq na equação de tau simbólica.
        """
        # Importante manter esta ordem!
        if ddq_d: tau = tau.subs(zip(self.ddq_d, ddq_d))
        if dq_d: tau = tau.subs(zip(self.dq_d, dq_d))
        if q_d: tau = tau.subs(zip(self.q_d, q_d))
        return tau.simplify()
    
    def eval_matrix(self, matrix, q=None, dq=None, ddq=None):
        """
        Substituição dos valores das variáveis q, dq e ddq em matrizes simbólicas.
        """
        # Importante manter esta ordem!
        if ddq: matrix = matrix.subs(zip(self.ddq, ddq))
        if dq: matrix = matrix.subs(zip(self.dq, dq))
        if q: matrix = matrix.subs(zip(self.q, q))
        return matrix.simplify()

