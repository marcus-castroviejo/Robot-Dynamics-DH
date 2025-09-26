"""Módulo de dinâmica do robô"""
from .robot_dynamics import Robot
from .cocoabot import CocoaBot
from .controller import CalculatedTorqueController

__all__ = ["Robot", "CocoaBot", "CalculatedTorqueController"]