"""
=================================================================================================================
                                    Gerenciador de Comunicação ESP32
=================================================================================================================
Camada de abstração sobre o servidor TCP que:
- Interpreta o protocolo de comunicação
- Fornece API de alto nível para envio/recebimento
- Gerencia sincronização e timeouts
- Processa respostas da ESP32

Separa a lógica de comunicação da interface gráfica e da simulação.
"""

from PyQt6.QtCore import QObject, pyqtSignal, QEventLoop, QTimer
import numpy as np
from typing import Optional, List
import logging

from .esp32_server import ESP32Server
from .protocol import (
    ProtocolBuilder, ProtocolParser, MeasurementData,
    ESP32Commands, ESP32Responses
)


class CommunicationManager(QObject):
    """
    Gerenciador de alto nível para comunicação com ESP32.
    
    Responsabilidades:
    - Gerenciar servidor TCP
    - Implementar protocolo de comandos
    - Sincronizar estados
    - Prover API simples para simulação
    """
    
    # ===== Sinais =====
    status_message = pyqtSignal(str)                 # Mensagens de status
    connection_changed = pyqtSignal(bool)             # True=conectado, False=desconectado
    measurement_received = pyqtSignal(object)         # MeasurementData recebido
    error_occurred = pyqtSignal(str)                  # Erros
    
    """
    =================================================================================================================
                                                Setup Inicial
    =================================================================================================================
    """
    
    def __init__(self, parent: Optional[QObject] = None):
        """Inicializa o gerenciador de comunicação"""
        super().__init__(parent)
        
        self._logger = logging.getLogger("rrp_app.comm_manager")
        
        # Servidor TCP
        self._server = ESP32Server(self)
        self._setup_server_signals()
        
        # Estado
        self._last_measurement: Optional[MeasurementData] = None
        self._measurement_sequence = 0  # Contador de medições recebidas
        
        # Configurações
        self._sync_timeout_s = 5.0      # Timeout para sincronização
        self._sync_tolerance = 0.01     # Tolerância para considerar sincronizado
    
    def _setup_server_signals(self):
        """Conecta sinais do servidor aos handlers"""
        self._server.status_changed.connect(self.status_message.emit)
        self._server.error_occurred.connect(self.error_occurred.emit)
        
        self._server.client_connected.connect(
            lambda ip, port: self._on_connection_changed(True, ip, port)
        )
        self._server.client_disconnected.connect(
            lambda: self._on_connection_changed(False)
        )
        
        self._server.json_received.connect(self._on_json_received)
        self._server.text_received.connect(self._on_text_received)

    """
    =================================================================================================================
                                                API Pública
    =================================================================================================================
    """

    def start_server(self, port: int = 9000) -> bool:
        """
        Inicia o servidor de comunicação
        
        Args:
            port: Porta TCP para escutar
            
        Returns:
            True se iniciou com sucesso
        """
        return self._server.start(port)
    
    def stop_server(self) -> None:
        """Para o servidor de comunicação"""
        self._server.stop()
    
    def is_connected(self) -> bool:
        """Verifica se ESP32 está conectada"""
        return self._server.is_client_connected()
    
    def send_ping(self) -> bool:
        """
        Envia comando de ping
        
        Returns:
            True se enviou com sucesso
        """
        cmd = ProtocolBuilder.build_ping()
        return self._server.send_json(cmd)
    
    def request_measurement(self) -> bool:
        """
        Solicita medição das juntas
        
        Returns:
            True se enviou com sucesso
        """
        cmd = ProtocolBuilder.build_get_measurement()
        return self._server.send_json(cmd)
    
    def send_reference(self, q_cmd: List[float], gripper: int) -> bool:
        """
        Envia comando de referência (posição das juntas + garra)
        
        Args:
            q_cmd: Comando de posição [q1, q2, d3]
            gripper: Posição da garra (graus)
            
        Returns:
            True se enviou com sucesso
        """
        cmd = ProtocolBuilder.build_set_reference(q_cmd, gripper)
        return self._server.send_json(cmd)
    
    def send_gripper_command(self, value: int) -> bool:
        """
        Envia comando apenas para a garra
        
        Args:
            value: Posição desejada (graus)
            
        Returns:
            True se enviou com sucesso
        """
        cmd = ProtocolBuilder.build_set_gripper(value)
        return self._server.send_json(cmd)
    
    def send_gains(self, Kp: List[float], Kd: List[float], Ki: List[float]) -> bool:
        """
        Envia ganhos do controlador PID para ESP32
        
        Args:
            Kp, Kd, Ki: Ganhos para cada junta (listas de 3 elementos)
            
        Returns:
            True se enviou com sucesso
        """
        cmd = ProtocolBuilder.build_set_gains(Kp, Kd, Ki)
        success = self._server.send_json(cmd)
        if success:
            self.status_message.emit(f"Ganhos enviados: Kp={Kp}, Kd={Kd}, Ki={Ki}")
        return success
    
    def send_reference_pid(self, q_ref: List[float], qd_ref: List[float], 
                          qdd_ref: List[float], gripper: int) -> bool:
        """
        Envia referência completa para controle PID de baixo nível
        
        Args:
            q_ref: Posição de referência
            qd_ref: Velocidade de referência
            qdd_ref: Aceleração de referência
            gripper: Posição da garra
            
        Returns:
            True se enviou com sucesso
        """
        cmd = ProtocolBuilder.build_set_reference_pid(q_ref, qd_ref, qdd_ref, gripper)
        return self._server.send_json(cmd)

    def get_last_measurement(self) -> Optional[MeasurementData]:
        """
        Retorna última medição recebida
        
        Returns:
            MeasurementData ou None se nenhuma medição disponível
        """
        return self._last_measurement
    
    def wait_for_measurement(self, timeout_ms: int = 100) -> bool:
        """
        Aguarda uma nova medição (bloqueante)
        
        Args:
            timeout_ms: Tempo máximo de espera em milissegundos
            
        Returns:
            True se recebeu nova medição, False se timeout
        """
        seq_before = self._measurement_sequence
        
        loop = QEventLoop()
        timer = QTimer()
        timer.setSingleShot(True)
        
        # Conecta sinais
        received = [False]
        
        def on_measurement(data):
            received[0] = True
            loop.quit()
        
        def on_timeout():
            loop.quit()
        
        self.measurement_received.connect(on_measurement)
        timer.timeout.connect(on_timeout)
        
        # Inicia timer e loop
        timer.start(timeout_ms)
        loop.exec()
        
        # Desconecta sinais
        self.measurement_received.disconnect(on_measurement)
        timer.timeout.disconnect(on_timeout)
        
        return received[0]
    
    def synchronize_initial_position(self, q0: List[float], gripper: int = 0) -> bool:
        """
        Sincroniza posição inicial com ESP32
        
        Envia comando de posição e aguarda até que a medição esteja dentro
        da tolerância ou até timeout.
        
        Args:
            q0: Posição inicial desejada [q1, q2, d3]
            gripper: Posição inicial da garra
            
        Returns:
            True se sincronizou com sucesso, False se timeout
        """
        self.status_message.emit("Sincronizando com ESP32...")
        
        q0_array = np.asarray(q0, dtype=float).reshape(3)
        
        from time import perf_counter
        t0 = perf_counter()
        
        while (perf_counter() - t0) < self._sync_timeout_s:
            # Envia comando
            self.send_reference(list(q0_array), gripper)
            
            # Solicita medição
            self.request_measurement()
            
            # Aguarda resposta
            if not self.wait_for_measurement(timeout_ms=100):
                continue  # Timeout, tenta novamente
            
            # Verifica erro
            if self._last_measurement is not None and self._last_measurement.is_valid():
                meas_array = np.asarray(self._last_measurement.q, dtype=float)
                error = np.linalg.norm(meas_array - q0_array)
                
                if error < self._sync_tolerance:
                    self.status_message.emit("ESP32 sincronizada")
                    return True
        
        # Timeout
        self.status_message.emit("Aviso: sincronização não confirmada")
        return False
    
    def set_sync_parameters(self, timeout_s: float = 5.0, tolerance: float = 0.01):
        """
        Configura parâmetros de sincronização
        
        Args:
            timeout_s: Timeout em segundos
            tolerance: Tolerância de erro de posição
        """
        self._sync_timeout_s = timeout_s
        self._sync_tolerance = tolerance

    """
    =================================================================================================================
                                            Handlers Internos (Privados)
    =================================================================================================================
    """
    
    def _on_connection_changed(self, connected: bool, ip: str = "", port: int = 0):
        """Handler para mudança de estado de conexão"""
        if connected:
            self.status_message.emit(f"ESP32 conectada: {ip}:{port}")
        else:
            self.status_message.emit("ESP32 desconectada")
            self._last_measurement = None
            self._measurement_sequence = 0
        
        self.connection_changed.emit(connected)
    
    def _on_text_received(self, text: str):
        """Handler para texto recebido (não-JSON)"""
        self._logger.debug(f"Texto recebido: {text}")
        self.status_message.emit(f"RX texto: {text}")
    
    def _on_json_received(self, obj: dict):
        """Handler para JSON recebido - interpreta protocolo"""
        
        # Pong (resposta a ping)
        if ProtocolParser.is_pong(obj):
            self.status_message.emit("RX: pong")
            return
        
        # Confirmação de referência
        if ProtocolParser.is_ref_ack(obj):
            ref_value = ProtocolParser.get_ref_value(obj)
            self.status_message.emit(f"RX: confirmação de referência")
            self._logger.debug(f"Referência confirmada: {ref_value}")
            return
        
        # Medição das juntas
        if ProtocolParser.is_measurement(obj):
            try:
                meas = ProtocolParser.parse_measurement(obj)
                
                if not meas.is_valid():
                    self._logger.warning("Medição inválida ignorada")
                    return
                
                self._last_measurement = meas
                self._measurement_sequence += 1
                
                self._logger.debug(f"Medição recebida: q={meas.q}, gripper={meas.gripper}")
                self.measurement_received.emit(meas)
                
            except Exception as e:
                self._logger.error(f"Erro ao processar medição: {e}")
                self.error_occurred.emit(f"Erro ao processar medição: {e}")
            return
        
        # Mensagem desconhecida
        self._logger.warning(f"Mensagem JSON desconhecida: {obj}")