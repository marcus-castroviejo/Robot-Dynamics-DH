"""
=================================================================================================================
                                        Servidor TCP para ESP32
=================================================================================================================
Servidor TCP não-bloqueante usando PyQt6 para comunicação com ESP32.
Responsável apenas pela camada de transporte (TCP/IP), não interpreta protocolo.

Protocolo de transporte: NDJSON (Newline Delimited JSON)
- Cada mensagem é uma linha terminada com '\n'
"""

from PyQt6.QtCore import QObject, pyqtSignal, QByteArray
from PyQt6.QtNetwork import QTcpServer, QTcpSocket, QAbstractSocket, QHostAddress
import json
import logging
from typing import Optional


class ESP32Server(QObject):
    """
    Servidor TCP não-bloqueante (Qt) para comunicação com ESP32.
    
    Responsabilidades:
    - Escutar em uma porta TCP
    - Aceitar conexão de um cliente ESP32
    - Enviar/receber linhas de texto
    - Serializar/deserializar JSON
    
    Não interpreta o conteúdo das mensagens (delegado ao CommunicationManager).
    """
    
    # ===== Sinais =====
    server_started = pyqtSignal(int)                 # porta
    server_stopped = pyqtSignal()
    client_connected = pyqtSignal(str, int)          # ip, porta
    client_disconnected = pyqtSignal()
    error_occurred = pyqtSignal(str)
    status_changed = pyqtSignal(str)
    
    # Sinais de dados
    text_received = pyqtSignal(str)                  # Linha de texto recebida
    json_received = pyqtSignal(dict)                 # Objeto JSON recebido

    """
    =================================================================================================================
                                                Setup Inicial
    =================================================================================================================
    """
    
    def __init__(self, parent: Optional[QObject] = None):
        """Inicializa o servidor TCP"""
        super().__init__(parent)
        
        self._logger = logging.getLogger("rrp_app.esp32_server")
        self._server = QTcpServer(self)
        self._server.newConnection.connect(self._on_new_connection)
        
        self._client: Optional[QTcpSocket] = None
        self._buffer = bytearray()

    """
    =================================================================================================================
                                                API Pública
    =================================================================================================================
    """

    def start(self, port: int = 9000) -> bool:
        """
        Inicia o servidor TCP
        
        Args:
            port: Porta para escutar (padrão: 9000)
            
        Returns:
            True se iniciou com sucesso, False caso contrário
        """
        if self._server.isListening():
            self.stop()

        ok = self._server.listen(QHostAddress.SpecialAddress.AnyIPv4, port)
        
        if ok:
            self._logger.info(f"Servidor ouvindo na porta {port}")
            self.status_changed.emit(f"Servidor ouvindo em 0.0.0.0:{port}")
            self.server_started.emit(port)
        else:
            err = self._server.errorString()
            msg = f"Falha ao iniciar servidor: {err}"
            self._logger.error(msg)
            self.status_changed.emit(msg)
            self.error_occurred.emit(msg)
            
        return ok

    def stop(self) -> None:
        """Para o servidor e desconecta cliente se houver"""
        if self._client is not None:
            try:
                self._client.disconnectFromHost()
                self._client.close()
            except Exception as e:
                self._logger.warning(f"Erro ao desconectar cliente: {e}")
            finally:
                self._client = None
                
        if self._server.isListening():
            self._server.close()
            self._logger.info("Servidor parado")
            self.status_changed.emit("Servidor parado")
            self.server_stopped.emit()
            
        self._buffer.clear()

    def is_listening(self) -> bool:
        """Verifica se o servidor está escutando"""
        return self._server.isListening()

    def is_client_connected(self) -> bool:
        """Verifica se há cliente conectado"""
        return (self._client is not None and 
                self._client.state() == QAbstractSocket.SocketState.ConnectedState)

    def send_text(self, text: str) -> bool:
        """
        Envia linha de texto para ESP32
        
        Args:
            text: Texto a enviar ('\n' será adicionado se necessário)
            
        Returns:
            True se enviou com sucesso, False caso contrário
        """
        if not self.is_client_connected():
            msg = "Não é possível enviar: nenhum cliente conectado"
            self._logger.warning(msg)
            self.error_occurred.emit(msg)
            return False

        if not text.endswith("\n"):
            text += "\n"

        try:
            data: QByteArray = QByteArray(text.encode("utf-8"))
            bytes_written = self._client.write(data)
            self._logger.debug(f"Enviado (texto): {text.strip()} ({bytes_written} bytes)")
            return bytes_written > 0
        except Exception as e:
            self._logger.error(f"Erro ao enviar texto: {e}")
            self.error_occurred.emit(f"Erro ao enviar: {e}")
            return False

    def send_json(self, payload: dict) -> bool:
        """
        Serializa e envia objeto JSON
        
        Args:
            payload: Dicionário a ser enviado como JSON
            
        Returns:
            True se enviou com sucesso, False caso contrário
        """
        try:
            line = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
        except Exception as e:
            self._logger.exception(f"Falha ao serializar JSON: {e}")
            self.error_occurred.emit(f"Falha ao serializar JSON: {e}")
            return False
            
        return self.send_text(line)

    """
    =================================================================================================================
                                            Handlers Internos (Privados)
    =================================================================================================================
    """

    def _on_new_connection(self) -> None:
        """Callback quando novo cliente conecta"""
        while self._server.hasPendingConnections():
            sock = self._server.nextPendingConnection()

            # Se já tem cliente, fecha o anterior
            if self._client is not None:
                try:
                    self._client.disconnectFromHost()
                    self._client.close()
                except Exception:
                    pass

            self._client = sock
            self._client.readyRead.connect(self._on_ready_read)
            self._client.disconnected.connect(self._on_client_disconnected)
            self._client.errorOccurred.connect(self._on_client_error)

            try:
                ip = self._client.peerAddress().toString()
                port = self._client.peerPort()
            except Exception:
                ip, port = "?", 0

            self._logger.info(f"Cliente conectado: {ip}:{port}")
            self.status_changed.emit(f"Cliente conectado: {ip}:{port}")
            self.client_connected.emit(ip, int(port))
            self._buffer.clear()

    def _on_client_disconnected(self) -> None:
        """Callback quando cliente desconecta"""
        self._logger.info("Cliente desconectado")
        self.status_changed.emit("Cliente desconectado")
        self.client_disconnected.emit()
        self._client = None
        self._buffer.clear()

    def _on_client_error(self, socket_error: QAbstractSocket.SocketError) -> None:
        """Callback quando ocorre erro no socket"""
        error_messages = {
            QAbstractSocket.SocketError.ConnectionRefusedError: "Conexão recusada",
            QAbstractSocket.SocketError.RemoteHostClosedError: "Cliente encerrou conexão",
            QAbstractSocket.SocketError.HostNotFoundError: "Host não encontrado",
            QAbstractSocket.SocketError.NetworkError: "Erro de rede",
            QAbstractSocket.SocketError.SocketTimeoutError: "Timeout",
        }
        
        msg = error_messages.get(socket_error, f"Erro de socket: {socket_error.name}")
        self._logger.error(msg)
        self.status_changed.emit(msg)
        self.error_occurred.emit(msg)

    def _on_ready_read(self) -> None:
        """Callback quando há dados para ler"""
        if self._client is None:
            return
            
        incoming: QByteArray = self._client.readAll()
        self._buffer.extend(bytes(incoming))

        # Parser linha por linha
        while True:
            try:
                idx = self._buffer.index(0x0A)  # '\n'
            except ValueError:
                break  # Não há linha completa ainda

            line_bytes = self._buffer[:idx]
            del self._buffer[:idx + 1]

            try:
                line = line_bytes.decode("utf-8", errors="replace").strip()
            except Exception as e:
                self._logger.exception(f"Falha ao decodificar linha: {e}")
                self.error_occurred.emit(f"Falha ao decodificar: {e}")
                continue

            if not line:
                continue

            # Tenta parsear como JSON
            try:
                obj = json.loads(line)
                if isinstance(obj, dict):
                    self._logger.debug(f"Recebido (JSON): {obj}")
                    self.json_received.emit(obj)
                else:
                    # JSON válido mas não é dict -> trata como texto
                    self._logger.debug(f"Recebido (JSON não-dict): {obj}")
                    self.text_received.emit(line)
            except json.JSONDecodeError:
                # Não é JSON -> emite como texto
                self._logger.debug(f"Recebido (texto): {line}")
                self.text_received.emit(line)