"""
Pacote de testes para o sistema CocoaBot

Estrutura:
- test_protocol.py: Testes do protocolo de comunicação
- test_comm_manager.py: Testes do gerenciador de comunicação
- test_esp32_server.py: Testes do servidor TCP
- test_simulation_thread.py: Testes da thread de simulação
- test_integration.py: Testes de integração end-to-end
- mock_esp32.py: Simulador de ESP32 para testes
- conftest.py: Fixtures compartilhadas do pytest

Como rodar:
    pytest tests/                    # Todos os testes
    pytest tests/test_protocol.py    # Teste específico
    pytest -v tests/                 # Modo verbose
    pytest --cov=communication tests/ # Com cobertura
"""

__version__ = "1.0.0"