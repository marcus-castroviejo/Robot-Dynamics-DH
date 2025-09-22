"""Módulo de interface do usuário"""
# from .main_window import RobotControlInterface
from .main_window_compact import RobotControlInterface
from .simulation_thread import SimulationThread

__all__ = ["RobotControlInterface", "SimulationThread"]