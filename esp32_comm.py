# esp32_comm.py
# Módulo de comunicação TCP com a ESP32 usando PyQt6 (QTcpSocket)
# Protocolo: linhas terminadas em '\n' (NDJSON: JSON por linha) ou texto simples por linha.

from PyQt6.QtCore import QObject, pyqtSignal, QByteArray
from PyQt6.QtNetwork import QAbstractSocket, QTcpSocket
import json
import logging


class ESP32Connection(QObject):
    """
    Classe responsável por gerenciar a conexão TCP com a ESP32, enviar/receber dados,
    e emitir sinais para a interface reagir (sem travar a GUI).
    """
    # ------- Sinais que a interface pode conectar -------
    connected = pyqtSignal()             # Emitido quando o socket conecta
    disconnected = pyqtSignal()          # Emitido quando o socket desconecta
    error_occurred = pyqtSignal(str)     # Emitido quando ocorre algum erro (mensagem amigável)
    text_received = pyqtSignal(str)      # Emitido quando chega uma linha de texto
    json_received = pyqtSignal(dict)     # Emitido quando chega uma linha com JSON válido
    status = pyqtSignal(str)             # Mensagens de status (para mostrar na barra da UI)

    def __init__(self, parent: QObject | None = None):
        """
        Construtor: cria o socket, conecta sinais e prepara o buffer de recepção.
        """
        super().__init__(parent)
        self._logger = logging.getLogger("rrp_app")  # usa o logger global configurado na main
        self._socket = QTcpSocket(self)              # socket TCP do Qt
        self._buffer = bytearray()                   # buffer onde juntamos bytes até formar linhas
        self._host = ""
        self._port = 0

        # Conecta sinais nativos do QTcpSocket aos nossos handlers
        self._socket.connected.connect(self._on_connected)
        self._socket.disconnected.connect(self._on_disconnected)
        self._socket.readyRead.connect(self._on_ready_read)
        self._socket.errorOccurred.connect(self._on_error)

    # ------------------- API pública -------------------

    def connect_to(self, host: str, port: int) -> None:
        """
        Inicia a conexão TCP à ESP32 (não bloqueia).
        host: endereço IP ou hostname da ESP32 (ex.: '192.168.4.1' no modo AP)
        port: porta TCP aberta na ESP32 (ex.: 3333)
        """
        self._host, self._port = host, port
        self._logger.info(f"Tentando conectar à ESP32 em {host}:{port}...")
        self.status.emit(f"Conectando a {host}:{port}...")
        self._socket.abort()  # cancela qualquer tentativa anterior
        self._socket.connectToHost(host, port)

    def disconnect(self) -> None:
        """
        Solicita o encerramento educado da conexão.
        """
        if self.is_connected():
            self._logger.info("Desconectando da ESP32...")
            self.status.emit("Desconectando...")
            self._socket.disconnectFromHost()
        else:
            self._socket.abort()  # garante que está tudo parado

    def is_connected(self) -> bool:
        """
        Retorna True se o socket estiver em estado conectado.
        """
        return self._socket.state() == QAbstractSocket.SocketState.ConnectedState

    def send_text(self, text: str) -> None:
        """
        Envia uma string como uma linha (terminada com '\n').
        Útil para comandos simples (ex.: 'LED ON').
        """
        if not self.is_connected():
            msg = "Não é possível enviar: socket não conectado."
            self._logger.warning(msg)
            self.error_occurred.emit(msg)
            return

        # Garante newline (protocolo de linha)
        if not text.endswith("\n"):
            text += "\n"

        data: QByteArray = QByteArray(text.encode("utf-8"))
        bytes_written = self._socket.write(data)
        self._logger.debug(f"Enviado (texto): {text.strip()} ({bytes_written} bytes)")
        self.status.emit(f"Enviado: {text.strip()}")

    def send_json(self, payload: dict) -> None:
        """
        Serializa e envia um dicionário como JSON por linha (NDJSON).
        Ex.: {"cmd":"set_ref","q":[0.1,0.2,0.3]}
        """
        try:
            line = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
        except Exception as e:
            self._logger.exception(f"Falha ao serializar JSON: {e}")
            self.error_occurred.emit(f"Falha ao serializar JSON: {e}")
            return

        self.send_text(line)

    # ------------------- Handlers privados -------------------

    def _on_connected(self) -> None:
        """
        Chamado automaticamente quando o QTcpSocket conecta.
        """
        self._logger.info(f"Conectado à ESP32 em {self._host}:{self._port}")
        self.status.emit(f"Conectado a {self._host}:{self._port}")
        self.connected.emit()

    def _on_disconnected(self) -> None:
        """
        Chamado automaticamente quando o QTcpSocket desconecta.
        """
        self._logger.info("Conexão com a ESP32 encerrada.")
        self.status.emit("Desconectado.")
        self.disconnected.emit()

    def _on_ready_read(self) -> None:
        """
        Chamado quando existem bytes disponíveis para leitura.
        Implementa um parser de linhas: acumula no buffer e separa por '\n'.
        Se a linha for JSON válido -> emite json_received(dict)
        Caso contrário -> emite text_received(str)
        """
        # Lê todos os bytes disponíveis e adiciona ao buffer
        incoming: QByteArray = self._socket.readAll()
        self._buffer.extend(bytes(incoming))

        # Processa por linhas (se houver mais de uma, entrega todas)
        while True:
            try:
                idx = self._buffer.index(0x0A)  # 0x0A == '\n'
            except ValueError:
                # Ainda não temos uma linha completa
                break

            # Pega a linha (sem o \n) e remove do buffer
            line_bytes = self._buffer[:idx]
            del self._buffer[:idx + 1]  # remove inclusive o '\n'

            # Decodifica UTF-8 com tolerância
            try:
                line = line_bytes.decode("utf-8", errors="replace").strip()
            except Exception as e:
                self._logger.exception(f"Falha ao decodificar linha: {e}")
                self.error_occurred.emit(f"Falha ao decodificar linha: {e}")
                continue

            if not line:
                continue  # ignora linhas vazias

            # Tenta interpretar como JSON; se falhar, entrega como texto
            try:
                obj = json.loads(line)
                if isinstance(obj, dict):
                    self._logger.debug(f"Recebido (JSON): {obj}")
                    self.json_received.emit(obj)
                else:
                    # Se for JSON válido mas não dict (ex.: lista), trate como texto
                    self._logger.debug(f"Recebido (JSON não-dict): {obj}")
                    self.text_received.emit(line)
            except json.JSONDecodeError:
                self._logger.debug(f"Recebido (texto): {line}")
                self.text_received.emit(line)

    def _on_error(self, socket_error: QAbstractSocket.SocketError) -> None:
        """
        Chamado quando ocorre um erro no socket. Emite mensagens amigáveis e loga o detalhe.
        """
        # Mapeia alguns erros comuns para mensagens mais claras
        mapping = {
            QAbstractSocket.SocketError.ConnectionRefusedError: "Conexão recusada (verifique IP/porta e se o servidor está ativo).",
            QAbstractSocket.SocketError.RemoteHostClosedError: "Conexão encerrada pelo host remoto.",
            QAbstractSocket.SocketError.HostNotFoundError: "Host não encontrado (verifique o endereço).",
            QAbstractSocket.SocketError.NetworkError: "Erro de rede (Wi-Fi instável ou fora da mesma sub-rede?).",
            QAbstractSocket.SocketError.SocketTimeoutError: "Tempo esgotado na operação de socket.",
        }
        human = mapping.get(socket_error, f"Erro de socket: {socket_error.name}")
        self._logger.error(f"{human} | Detalhe Qt: {socket_error.name}")
        self.status.emit(human)
        self.error_occurred.emit(human)
