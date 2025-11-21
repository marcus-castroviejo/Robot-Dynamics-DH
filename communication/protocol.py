"""
=================================================================================================================
                                        Protocolo de Comunicação ESP32
=================================================================================================================
Define comandos, tipos de mensagens e estrutura do protocolo de comunicação
entre o PC e a ESP32.

Protocolo: NDJSON (Newline Delimited JSON)
- Cada mensagem é um objeto JSON terminado com '\n'
- Comunicação bidirecional: PC ↔ ESP32
"""

from typing import Dict, List, Any
from dataclasses import dataclass


class ESP32Commands:
    """Comandos disponíveis no protocolo"""
    
    # Comandos de controle
    PING = "ping"
    GET_MEAS = "get_meas"
    SET_REF = "set_ref"
    SET_GRIPPER = "set_gripper"
    
    # Comandos de configuração (futuros)
    SET_CONFIG = "set_config"
    GET_STATUS = "get_status"


class ESP32Responses:
    """Tipos de resposta esperados da ESP32"""
    
    PONG = "pong"
    REF_ACK = "ref"
    MEASUREMENT = "meas_q"
    STATUS = "status"
    ERROR = "error"


@dataclass
class MeasurementData:
    """Dados de medição recebidos da ESP32"""
    q: List[float]          # Posições das juntas [q1, q2, d3]
    gripper: int            # Posição da garra (graus)
    timestamp: float        # Timestamp da medição
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'MeasurementData':
        """Cria MeasurementData a partir de um dicionário"""
        return cls(
            q=data.get("meas_q", [0.0, 0.0, 0.0]),
            gripper=data.get("meas_gripper", 0),
            timestamp=data.get("t", 0.0)
        )
    
    def is_valid(self) -> bool:
        """Verifica se os dados são válidos"""
        if len(self.q) != 3:
            return False
        # Filtro anti-burst: ignora medições zeradas
        if all(abs(val) < 1e-9 for val in self.q):
            return False
        return True


class ProtocolBuilder:
    """Construtor de mensagens do protocolo"""
    
    @staticmethod
    def build_ping() -> Dict[str, Any]:
        """Constrói mensagem de ping"""
        return {"cmd": ESP32Commands.PING}
    
    @staticmethod
    def build_get_measurement() -> Dict[str, Any]:
        """Constrói mensagem de requisição de medição"""
        return {"cmd": ESP32Commands.GET_MEAS}
    
    @staticmethod
    def build_set_reference(q_cmd: List[float], gripper: int) -> Dict[str, Any]:
        """
        Constrói mensagem de definição de referência
        
        Args:
            q_cmd: Comando de posição das juntas [q1, q2, d3]
            gripper: Posição da garra (graus)
        """
        return {
            "cmd": ESP32Commands.SET_REF,
            "q_cmd": list(q_cmd),
            "gripper": int(gripper)
        }
    
    @staticmethod
    def build_set_gripper(value: int) -> Dict[str, Any]:
        """
        Constrói mensagem de controle da garra
        
        Args:
            value: Posição desejada da garra (graus)
        """
        return {
            "cmd": ESP32Commands.SET_GRIPPER,
            "value": int(value)
        }


class ProtocolParser:
    """Parser de mensagens recebidas"""
    
    @staticmethod
    def is_pong(message: Dict[str, Any]) -> bool:
        """Verifica se a mensagem é um pong"""
        return ESP32Responses.PONG in message
    
    @staticmethod
    def is_ref_ack(message: Dict[str, Any]) -> bool:
        """Verifica se a mensagem é confirmação de referência"""
        return ESP32Responses.REF_ACK in message
    
    @staticmethod
    def is_measurement(message: Dict[str, Any]) -> bool:
        """Verifica se a mensagem contém dados de medição"""
        return ESP32Responses.MEASUREMENT in message
    
    @staticmethod
    def parse_measurement(message: Dict[str, Any]) -> MeasurementData:
        """
        Extrai dados de medição de uma mensagem
        
        Returns:
            MeasurementData ou None se inválido
        """
        if not ProtocolParser.is_measurement(message):
            raise ValueError("Mensagem não contém dados de medição")
        
        return MeasurementData.from_dict(message)
    
    @staticmethod
    def get_ref_value(message: Dict[str, Any]) -> Any:
        """Extrai valor de confirmação de referência"""
        return message.get(ESP32Responses.REF_ACK)