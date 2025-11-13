# esp32_comm.py
# Servidor TCP (PC) para conversar com a ESP32 via PyQt6.
# Protocolo: 1 mensagem por linha, terminada com '\n' (NDJSON: JSON por linha).

from PyQt6.QtCore import QObject, pyqtSignal, QByteArray
from PyQt6.QtNetwork import QTcpServer, QTcpSocket, QAbstractSocket, QHostAddress
import json
import logging


class ESP32Server(QObject):
    """
    Servidor TCP não-bloqueante (Qt) que:
      - escuta em uma porta (ex.: 9000)
      - aceita 1 cliente ESP32
      - recebe linhas e emite text_received/json_received
      - permite enviar texto/JSON para a ESP32
    """
    # ---- Sinais para a interface ----
    server_started = pyqtSignal(int)                 # porta
    server_stopped = pyqtSignal()
    client_connected = pyqtSignal(str, int)          # ip, porta
    client_disconnected = pyqtSignal()
    error_occurred = pyqtSignal(str)
    status = pyqtSignal(str)
    text_received = pyqtSignal(str)
    json_received = pyqtSignal(dict)

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        self._logger = logging.getLogger("rrp_app")
        self._server = QTcpServer(self)
        self._server.newConnection.connect(self._on_new_connection)

        self._client: QTcpSocket | None = None
        self._buffer = bytearray()

    # ------------- API pública -------------

    def start(self, port: int = 9000) -> bool:
        """Começa a escutar em 0.0.0.0:port (todas as interfaces IPv4)."""
        if self._server.isListening():
            self.stop()

        ok = self._server.listen(QHostAddress.SpecialAddress.AnyIPv4, port)
        if ok:
            self._logger.info(f"Servidor ouvindo na porta {port}")
            self.status.emit(f"Servidor ouvindo em 0.0.0.0:{port}")
            self.server_started.emit(port)
        else:
            err = self._server.errorString()
            msg = f"Falha ao iniciar servidor: {err}"
            self._logger.error(msg)
            self.status.emit(msg)
            self.error_occurred.emit(msg)
        return ok

    def stop(self) -> None:
        """Para de escutar e desconecta cliente, se houver."""
        if self._client is not None:
            try:
                self._client.disconnectFromHost()
                self._client.close()
            except Exception:
                pass
            self._client = None
        if self._server.isListening():
            self._server.close()
            self._logger.info("Servidor parado.")
            self.status.emit("Servidor parado.")
            self.server_stopped.emit()
        self._buffer.clear()

    def is_listening(self) -> bool:
        return self._server.isListening()

    def is_client_connected(self) -> bool:
        return self._client is not None and self._client.state() == QAbstractSocket.SocketState.ConnectedState

    def send_text(self, text: str) -> None:
        """Envia uma linha de texto para a ESP32 (com '\\n')."""
        if not self.is_client_connected():
            msg = "Não é possível enviar: nenhum cliente conectado."
            self._logger.warning(msg)
            self.error_occurred.emit(msg)
            return

        if not text.endswith("\n"):
            text += "\n"

        data: QByteArray = QByteArray(text.encode("utf-8"))
        bytes_written = self._client.write(data)
        self._logger.debug(f"Enviado (texto): {text.strip()} ({bytes_written} bytes)")
        #self.status.emit(f"Enviado: {text.strip()}")

    def send_json(self, payload: dict) -> None:
        """Serializa e envia um dicionário como JSON por linha (NDJSON)."""
        try:
            line = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
        except Exception as e:
            self._logger.exception(f"Falha ao serializar JSON: {e}")
            self.error_occurred.emit(f"Falha ao serializar JSON: {e}")
            return
        self.send_text(line)

    # ------------- Handlers internos -------------

    def _on_new_connection(self) -> None:
        # Aceita um único cliente; se já tem um, fecha o anterior e assume o novo.
        while self._server.hasPendingConnections():
            sock = self._server.nextPendingConnection()

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
            self.status.emit(f"Cliente conectado: {ip}:{port}")
            self.client_connected.emit(ip, int(port))
            self._buffer.clear()

    def _on_client_disconnected(self) -> None:
        self._logger.info("Cliente desconectado.")
        self.status.emit("Cliente desconectado.")
        self.client_disconnected.emit()
        self._client = None
        self._buffer.clear()

    def _on_client_error(self, socket_error: QAbstractSocket.SocketError) -> None:
        mapping = {
            QAbstractSocket.SocketError.ConnectionRefusedError: "Conexão recusada pelo cliente.",
            QAbstractSocket.SocketError.RemoteHostClosedError: "Cliente encerrou a conexão.",
            QAbstractSocket.SocketError.HostNotFoundError: "Cliente (host) não encontrado.",
            QAbstractSocket.SocketError.NetworkError: "Erro de rede.",
            QAbstractSocket.SocketError.SocketTimeoutError: "Tempo esgotado na operação de socket.",
        }
        human = mapping.get(socket_error, f"Erro de socket: {socket_error.name}")
        self._logger.error(human)
        self.status.emit(human)
        self.error_occurred.emit(human)

    def _on_ready_read(self) -> None:
        if self._client is None:
            return
        incoming: QByteArray = self._client.readAll()
        self._buffer.extend(bytes(incoming))

        # parser por linhas
        while True:
            try:
                idx = self._buffer.index(0x0A)  # '\n'
            except ValueError:
                break

            line_bytes = self._buffer[:idx]
            del self._buffer[:idx + 1]

            try:
                line = line_bytes.decode("utf-8", errors="replace").strip()
            except Exception as e:
                self._logger.exception(f"Falha ao decodificar linha: {e}")
                self.error_occurred.emit(f"Falha ao decodificar linha: {e}")
                continue

            if not line:
                continue

            # tenta JSON; senão, texto
            try:
                obj = json.loads(line)
                if isinstance(obj, dict):
                    self._logger.debug(f"Recebido (JSON): {obj}")
                    self.json_received.emit(obj)
                else:
                    # JSON válido mas não dict -> trate como texto
                    self._logger.debug(f"Recebido (JSON não-dict): {obj}")
                    self.text_received.emit(line)
            except json.JSONDecodeError:
                self._logger.debug(f"Recebido (texto): {line}")
                self.text_received.emit(line)
