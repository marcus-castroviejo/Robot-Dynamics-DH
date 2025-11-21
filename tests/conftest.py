"""
Configuração e fixtures compartilhadas para pytest

Este arquivo é automaticamente carregado pelo pytest e disponibiliza
fixtures para todos os testes.
"""
import pytest
import sys
from pathlib import Path

# Adiciona o diretório raiz ao path para imports
root_dir = Path(__file__).parent.parent
sys.path.insert(0, str(root_dir))


@pytest.fixture
def sample_measurement_valid():
    """Fixture com dados de medição válidos"""
    return {
        "meas_q": [0.1, 1.57, 0.03],
        "meas_gripper": 45,
        "t": 1.23
    }


@pytest.fixture
def sample_measurement_invalid_zeros():
    """Fixture com medição inválida (zeros)"""
    return {
        "meas_q": [0.0, 0.0, 0.0],
        "meas_gripper": 0,
        "t": 0.0
    }


@pytest.fixture
def sample_measurement_invalid_short():
    """Fixture com medição inválida (array curto)"""
    return {
        "meas_q": [0.1, 1.57],  # Faltando d3
        "meas_gripper": 45,
        "t": 1.23
    }


@pytest.fixture
def sample_trajectory_point():
    """Fixture com ponto de trajetória válido"""
    return {
        "t": 1.0,
        "q": [0.0, 1.57, 0.03],
        "qd": [0.1, 0.2, 0.01],
        "qdd": [0.05, 0.1, 0.005]
    }


@pytest.fixture
def sample_reference_command():
    """Fixture com comando de referência esperado"""
    return {
        "cmd": "set_ref",
        "q_cmd": [0.0, 1.57, 0.03],
        "gripper": 45
    }


@pytest.fixture
def sample_gripper_command():
    """Fixture com comando de garra esperado"""
    return {
        "cmd": "set_gripper",
        "value": 90
    }


@pytest.fixture
def sample_ping_command():
    """Fixture com comando de ping"""
    return {"cmd": "ping"}


@pytest.fixture
def sample_pong_response():
    """Fixture com resposta pong"""
    return {"pong": True}


@pytest.fixture
def tolerance():
    """Tolerância padrão para comparações numéricas"""
    return 1e-6