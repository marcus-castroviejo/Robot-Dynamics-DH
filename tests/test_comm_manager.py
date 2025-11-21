"""
Testes para o CommunicationManager

Testa a API de alto nível de comunicação usando mocks do ESP32Server.
Não requer hardware ou rede real.
"""
import pytest
from unittest.mock import Mock, MagicMock, patch, call
import numpy as np
from PyQt6.QtCore import QObject

from communication import (
    CommunicationManager,
    MeasurementData,
    ProtocolBuilder
)


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture
def mock_server():
    """Mock do ESP32Server"""
    server = Mock()
    server.is_client_connected.return_value = True
    server.send_json.return_value = True
    server.start.return_value = True
    return server


@pytest.fixture
def comm_manager(mock_server, qtbot):
    """Instância de CommunicationManager com servidor mockado"""
    with patch('communication.comm_manager.ESP32Server', return_value=mock_server):
        manager = CommunicationManager()
        # qtbot adiciona suporte para Qt signals
        qtbot.addWidget(manager)
        return manager


# =============================================================================
# Testes de Inicialização
# =============================================================================

class TestInitialization:
    """Testa inicialização do gerenciador"""
    
    def test_creates_server(self, qtbot):
        """Verifica que CommunicationManager cria ESP32Server"""
        with patch('communication.comm_manager.ESP32Server') as MockServer:
            manager = CommunicationManager()
            qtbot.addWidget(manager)
            MockServer.assert_called_once()
    
    def test_initial_state(self, comm_manager):
        """Verifica estado inicial"""
        assert comm_manager._last_measurement is None
        assert comm_manager._measurement_sequence == 0


# =============================================================================
# Testes de Controle do Servidor
# =============================================================================

class TestServerControl:
    """Testa controle do servidor TCP"""
    
    def test_start_server(self, comm_manager, mock_server):
        """Testa inicialização do servidor"""
        result = comm_manager.start_server(port=9000)
        
        assert result == True
        mock_server.start.assert_called_once_with(9000)
    
    def test_stop_server(self, comm_manager, mock_server):
        """Testa parada do servidor"""
        comm_manager.stop_server()
        
        mock_server.stop.assert_called_once()
    
    def test_is_connected_true(self, comm_manager, mock_server):
        """Testa verificação de conexão (conectado)"""
        mock_server.is_client_connected.return_value = True
        
        assert comm_manager.is_connected() == True
    
    def test_is_connected_false(self, comm_manager, mock_server):
        """Testa verificação de conexão (desconectado)"""
        mock_server.is_client_connected.return_value = False
        
        assert comm_manager.is_connected() == False


# =============================================================================
# Testes de Envio de Comandos
# =============================================================================

class TestSendCommands:
    """Testa envio de comandos"""
    
    def test_send_ping(self, comm_manager, mock_server):
        """Testa envio de ping"""
        result = comm_manager.send_ping()
        
        assert result == True
        mock_server.send_json.assert_called_once()
        
        # Verifica comando enviado
        sent_cmd = mock_server.send_json.call_args[0][0]
        assert sent_cmd["cmd"] == "ping"
    
    def test_request_measurement(self, comm_manager, mock_server):
        """Testa requisição de medição"""
        result = comm_manager.request_measurement()
        
        assert result == True
        mock_server.send_json.assert_called_once()
        
        # Verifica comando
        sent_cmd = mock_server.send_json.call_args[0][0]
        assert sent_cmd["cmd"] == "get_meas"
    
    def test_send_reference(self, comm_manager, mock_server):
        """Testa envio de referência"""
        q_cmd = [0.0, 1.57, 0.03]
        gripper = 45
        
        result = comm_manager.send_reference(q_cmd, gripper)
        
        assert result == True
        mock_server.send_json.assert_called_once()
        
        # Verifica comando
        sent_cmd = mock_server.send_json.call_args[0][0]
        assert sent_cmd["cmd"] == "set_ref"
        assert sent_cmd["q_cmd"] == q_cmd
        assert sent_cmd["gripper"] == gripper
    
    def test_send_gripper_command(self, comm_manager, mock_server):
        """Testa comando de garra"""
        value = 90
        
        result = comm_manager.send_gripper_command(value)
        
        assert result == True
        mock_server.send_json.assert_called_once()
        
        # Verifica comando
        sent_cmd = mock_server.send_json.call_args[0][0]
        assert sent_cmd["cmd"] == "set_gripper"
        assert sent_cmd["value"] == value


# =============================================================================
# Testes de Recepção de Dados
# =============================================================================

class TestReceiveData:
    """Testa recepção e processamento de dados"""
    
    def test_get_last_measurement_none(self, comm_manager):
        """Testa get_last_measurement quando não há medição"""
        assert comm_manager.get_last_measurement() is None
    
    def test_on_json_received_pong(self, comm_manager, qtbot):
        """Testa recepção de pong"""
        with qtbot.waitSignal(comm_manager.status_message, timeout=1000):
            comm_manager._on_json_received({"pong": True})
    
    def test_on_json_received_ref_ack(self, comm_manager, qtbot):
        """Testa recepção de confirmação de referência"""
        with qtbot.waitSignal(comm_manager.status_message, timeout=1000):
            comm_manager._on_json_received({"ref": [0.0, 1.57, 0.03]})
    
    def test_on_json_received_measurement_valid(self, comm_manager, qtbot, 
                                                 sample_measurement_valid):
        """Testa recepção de medição válida"""
        with qtbot.waitSignal(comm_manager.measurement_received, timeout=1000) as blocker:
            comm_manager._on_json_received(sample_measurement_valid)
        
        # Verifica sinal emitido
        meas = blocker.args[0]
        assert isinstance(meas, MeasurementData)
        assert meas.q == [0.1, 1.57, 0.03]
        assert meas.gripper == 45
        
        # Verifica armazenamento
        assert comm_manager.get_last_measurement() is not None
        assert comm_manager._measurement_sequence == 1
    
    def test_on_json_received_measurement_invalid(self, comm_manager, qtbot,
                                                   sample_measurement_invalid_zeros):
        """Testa que medição inválida é ignorada"""
        # Não deve emitir sinal
        comm_manager._on_json_received(sample_measurement_invalid_zeros)
        
        # Não deve armazenar
        assert comm_manager.get_last_measurement() is None
        assert comm_manager._measurement_sequence == 0


# =============================================================================
# Testes de Sincronização
# =============================================================================

class TestSynchronization:
    """Testa sincronização de posição inicial"""
    
    def test_set_sync_parameters(self, comm_manager):
        """Testa configuração de parâmetros de sync"""
        comm_manager.set_sync_parameters(timeout_s=10.0, tolerance=0.02)
        
        assert comm_manager._sync_timeout_s == 10.0
        assert comm_manager._sync_tolerance == 0.02
    
    @pytest.mark.skip(reason="Requer implementação completa de wait_for_measurement")
    def test_synchronize_success(self, comm_manager, mock_server, qtbot):
        """Testa sincronização bem-sucedida"""
        q0 = [0.0, 0.0, 0.0]
        gripper = 0
        
        # Mock para simular medição próxima de q0
        def simulate_measurement(*args, **kwargs):
            meas = MeasurementData(q=[0.0001, 0.0001, 0.0001], 
                                  gripper=0, timestamp=0.0)
            comm_manager._last_measurement = meas
            comm_manager._measurement_sequence += 1
            comm_manager.measurement_received.emit(meas)
            return True
        
        with patch.object(comm_manager, 'wait_for_measurement', 
                         side_effect=simulate_measurement):
            result = comm_manager.synchronize_initial_position(q0, gripper)
        
        assert result == True
    
    @pytest.mark.skip(reason="Requer implementação completa de wait_for_measurement")
    def test_synchronize_timeout(self, comm_manager, mock_server):
        """Testa timeout de sincronização"""
        q0 = [0.0, 0.0, 0.0]
        gripper = 0
        
        # Configura timeout curto
        comm_manager.set_sync_parameters(timeout_s=0.1, tolerance=0.01)
        
        # Mock que sempre retorna False (timeout)
        with patch.object(comm_manager, 'wait_for_measurement', return_value=False):
            result = comm_manager.synchronize_initial_position(q0, gripper)
        
        assert result == False


# =============================================================================
# Testes de Sinais Qt
# =============================================================================

class TestSignals:
    """Testa emissão de sinais Qt"""
    
    def test_status_message_signal(self, comm_manager, qtbot):
        """Testa sinal de status"""
        with qtbot.waitSignal(comm_manager.status_message, timeout=1000) as blocker:
            comm_manager.status_message.emit("Test message")
        
        assert blocker.args[0] == "Test message"
    
    def test_error_occurred_signal(self, comm_manager, qtbot):
        """Testa sinal de erro"""
        with qtbot.waitSignal(comm_manager.error_occurred, timeout=1000) as blocker:
            comm_manager.error_occurred.emit("Test error")
        
        assert blocker.args[0] == "Test error"
    
    def test_connection_changed_signal(self, comm_manager, qtbot):
        """Testa sinal de mudança de conexão"""
        with qtbot.waitSignal(comm_manager.connection_changed, timeout=1000) as blocker:
            comm_manager.connection_changed.emit(True)
        
        assert blocker.args[0] == True


# =============================================================================
# Testes de Integração (com Mock)
# =============================================================================

class TestIntegrationWithMock:
    """Testa fluxos completos com mock"""
    
    def test_full_cycle_send_receive(self, comm_manager, mock_server, qtbot,
                                     sample_measurement_valid):
        """Testa ciclo completo: envia comando → recebe medição"""
        # 1. Envia comando de referência
        q_cmd = [0.0, 1.57, 0.03]
        comm_manager.send_reference(q_cmd, gripper=45)
        
        # 2. Solicita medição
        comm_manager.request_measurement()
        
        # 3. Simula recepção de medição
        with qtbot.waitSignal(comm_manager.measurement_received, timeout=1000):
            comm_manager._on_json_received(sample_measurement_valid)
        
        # 4. Verifica medição armazenada
        meas = comm_manager.get_last_measurement()
        assert meas is not None
        assert meas.is_valid()


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])