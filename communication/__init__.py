"""
=================================================================================================================
                                        Pacote de Comunicação ESP32
=================================================================================================================
Gerencia toda a comunicação entre o PC e a ESP32.

Componentes:
- esp32_server: Servidor TCP (camada de transporte)
- protocol: Definição do protocolo de comandos/mensagens
- comm_manager: Gerenciador de alto nível (API principal)

Uso típico:
    from communication import CommunicationManager
    
    comm = CommunicationManager()
    comm.start_server(port=9000)
    comm.send_reference([0.0, 1.57, 0.03], gripper=45)
"""

from .esp32_server import ESP32Server
from .protocol import (
    ESP32Commands,
    ESP32Responses,
    MeasurementData,
    ProtocolBuilder,
    ProtocolParser
)
from .comm_manager import CommunicationManager

__all__ = [
    "ESP32Server",
    "ESP32Commands",
    "ESP32Responses",
    "MeasurementData",
    "ProtocolBuilder",
    "ProtocolParser",
    "CommunicationManager"
]