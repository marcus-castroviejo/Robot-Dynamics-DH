"""Módulo de dinâmica do robô"""
from .robot_dynamics import Robot
from .cocoabot import CocoaBot
from .controller import Controller

__all__ = ["Robot", "CocoaBot", "Controller"]