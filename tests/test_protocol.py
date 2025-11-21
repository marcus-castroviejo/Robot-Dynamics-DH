"""
Testes para o módulo de protocolo de comunicação

Testa:
- Builders de comandos (ProtocolBuilder)
- Parsers de respostas (ProtocolParser)
- Validação de dados (MeasurementData)
- Constantes de comandos e respostas

Estes testes não dependem de rede, Qt ou hardware.
São rápidos e podem rodar sempre.
"""
import pytest
import numpy as np
from communication import (
    ESP32Commands,
    ESP32Responses,
    MeasurementData,
    ProtocolBuilder,
    ProtocolParser
)


# =============================================================================
# Testes de Constantes
# =============================================================================

class TestConstants:
    """Testa constantes do protocolo"""
    
    def test_esp32_commands_exist(self):
        """Verifica que comandos estão definidos"""
        assert hasattr(ESP32Commands, 'PING')
        assert hasattr(ESP32Commands, 'GET_MEAS')
        assert hasattr(ESP32Commands, 'SET_REF')
        assert hasattr(ESP32Commands, 'SET_GRIPPER')
    
    def test_esp32_responses_exist(self):
        """Verifica que respostas estão definidas"""
        assert hasattr(ESP32Responses, 'PONG')
        assert hasattr(ESP32Responses, 'MEASUREMENT')
        assert hasattr(ESP32Responses, 'REF_ACK')
    
    def test_command_values(self):
        """Verifica valores dos comandos"""
        assert ESP32Commands.PING == "ping"
        assert ESP32Commands.GET_MEAS == "get_meas"
        assert ESP32Commands.SET_REF == "set_ref"
        assert ESP32Commands.SET_GRIPPER == "set_gripper"


# =============================================================================
# Testes de ProtocolBuilder
# =============================================================================

class TestProtocolBuilder:
    """Testa construção de comandos"""
    
    def test_build_ping(self, sample_ping_command):
        """Testa construção de comando ping"""
        cmd = ProtocolBuilder.build_ping()
        assert cmd == sample_ping_command
        assert cmd["cmd"] == "ping"
    
    def test_build_get_measurement(self):
        """Testa construção de comando get_meas"""
        cmd = ProtocolBuilder.build_get_measurement()
        assert cmd["cmd"] == "get_meas"
    
    def test_build_set_reference(self, sample_reference_command):
        """Testa construção de comando set_ref"""
        q_cmd = [0.0, 1.57, 0.03]
        gripper = 45
        
        cmd = ProtocolBuilder.build_set_reference(q_cmd, gripper)
        
        assert cmd == sample_reference_command
        assert cmd["cmd"] == "set_ref"
        assert cmd["q_cmd"] == q_cmd
        assert cmd["gripper"] == gripper
    
    def test_build_set_reference_with_numpy(self):
        """Testa set_ref com numpy array"""
        q_cmd = np.array([0.0, 1.57, 0.03])
        gripper = 45
        
        cmd = ProtocolBuilder.build_set_reference(q_cmd, gripper)
        
        assert cmd["cmd"] == "set_ref"
        assert isinstance(cmd["q_cmd"], list)
        assert len(cmd["q_cmd"]) == 3
    
    def test_build_set_gripper(self, sample_gripper_command):
        """Testa construção de comando set_gripper"""
        value = 90
        
        cmd = ProtocolBuilder.build_set_gripper(value)
        
        assert cmd == sample_gripper_command
        assert cmd["cmd"] == "set_gripper"
        assert cmd["value"] == value
    
    def test_build_set_gripper_type_conversion(self):
        """Testa conversão de tipo em set_gripper"""
        # Testa com float
        cmd = ProtocolBuilder.build_set_gripper(90.5)
        assert isinstance(cmd["value"], int)
        assert cmd["value"] == 90


# =============================================================================
# Testes de ProtocolParser
# =============================================================================

class TestProtocolParser:
    """Testa interpretação de respostas"""
    
    def test_is_pong(self, sample_pong_response):
        """Testa detecção de pong"""
        assert ProtocolParser.is_pong(sample_pong_response) == True
        assert ProtocolParser.is_pong({"other": "data"}) == False
    
    def test_is_ref_ack(self):
        """Testa detecção de confirmação de referência"""
        ref_msg = {"ref": [0.0, 1.57, 0.03]}
        other_msg = {"other": "data"}
        
        assert ProtocolParser.is_ref_ack(ref_msg) == True
        assert ProtocolParser.is_ref_ack(other_msg) == False
    
    def test_is_measurement(self, sample_measurement_valid):
        """Testa detecção de medição"""
        other_msg = {"other": "data"}
        
        assert ProtocolParser.is_measurement(sample_measurement_valid) == True
        assert ProtocolParser.is_measurement(other_msg) == False
    
    def test_parse_measurement_valid(self, sample_measurement_valid):
        """Testa parser de medição válida"""
        meas = ProtocolParser.parse_measurement(sample_measurement_valid)
        
        assert isinstance(meas, MeasurementData)
        assert meas.q == [0.1, 1.57, 0.03]
        assert meas.gripper == 45
        assert meas.timestamp == 1.23
    
    def test_parse_measurement_invalid_raises(self):
        """Testa que parser lança exceção para mensagem inválida"""
        invalid_msg = {"other": "data"}
        
        with pytest.raises(ValueError):
            ProtocolParser.parse_measurement(invalid_msg)
    
    def test_get_ref_value(self):
        """Testa extração de valor de referência"""
        ref_msg = {"ref": [0.0, 1.57, 0.03]}
        
        value = ProtocolParser.get_ref_value(ref_msg)
        assert value == [0.0, 1.57, 0.03]


# =============================================================================
# Testes de MeasurementData
# =============================================================================

class TestMeasurementData:
    """Testa estrutura de dados de medição"""
    
    def test_create_measurement(self):
        """Testa criação de MeasurementData"""
        meas = MeasurementData(
            q=[0.1, 1.57, 0.03],
            gripper=45,
            timestamp=1.23
        )
        
        assert meas.q == [0.1, 1.57, 0.03]
        assert meas.gripper == 45
        assert meas.timestamp == 1.23
    
    def test_from_dict_valid(self, sample_measurement_valid):
        """Testa criação a partir de dicionário"""
        meas = MeasurementData.from_dict(sample_measurement_valid)
        
        assert meas.q == [0.1, 1.57, 0.03]
        assert meas.gripper == 45
        assert meas.timestamp == 1.23
    
    def test_from_dict_missing_fields(self):
        """Testa from_dict com campos faltando (usa defaults)"""
        incomplete = {"meas_q": [0.1, 1.57, 0.03]}
        
        meas = MeasurementData.from_dict(incomplete)
        
        assert meas.q == [0.1, 1.57, 0.03]
        assert meas.gripper == 0  # Default
        assert meas.timestamp == 0.0  # Default
    
    def test_is_valid_true(self):
        """Testa validação de medição válida"""
        meas = MeasurementData(
            q=[0.1, 1.57, 0.03],
            gripper=45,
            timestamp=1.23
        )
        
        assert meas.is_valid() == True
    
    def test_is_valid_false_zeros(self, sample_measurement_invalid_zeros):
        """Testa validação com zeros (inválido)"""
        meas = MeasurementData.from_dict(sample_measurement_invalid_zeros)
        
        assert meas.is_valid() == False
    
    def test_is_valid_false_short_array(self):
        """Testa validação com array curto (inválido)"""
        meas = MeasurementData(
            q=[0.1, 1.57],  # Faltando d3
            gripper=45,
            timestamp=1.23
        )
        
        assert meas.is_valid() == False
    
    def test_is_valid_false_long_array(self):
        """Testa validação com array longo (inválido)"""
        meas = MeasurementData(
            q=[0.1, 1.57, 0.03, 0.5],  # Extra
            gripper=45,
            timestamp=1.23
        )
        
        assert meas.is_valid() == False
    
    def test_is_valid_nearly_zero(self):
        """Testa validação com valores quase zero (edge case)"""
        # Valores muito pequenos mas não exatamente zero
        meas = MeasurementData(
            q=[1e-10, 1e-10, 1e-10],
            gripper=0,
            timestamp=0.0
        )
        
        # Deve ser considerado inválido (filtro anti-burst)
        assert meas.is_valid() == False
        
        # Valores pequenos mas acima da tolerância
        meas2 = MeasurementData(
            q=[1e-8, 1e-8, 1e-8],
            gripper=0,
            timestamp=0.0
        )
        
        # Deve ser válido
        assert meas2.is_valid() == True


# =============================================================================
# Testes de Integração do Protocolo
# =============================================================================

class TestProtocolIntegration:
    """Testa fluxos completos do protocolo"""
    
    def test_build_parse_cycle(self):
        """Testa ciclo completo: build → send → receive → parse"""
        # 1. Build comando
        q_cmd = [0.0, 1.57, 0.03]
        gripper = 45
        cmd = ProtocolBuilder.build_set_reference(q_cmd, gripper)
        
        # 2. Simula envio/recepção (aqui apenas verifica estrutura)
        assert "cmd" in cmd
        assert "q_cmd" in cmd
        assert "gripper" in cmd
        
        # 3. Simula resposta da ESP32
        response = {
            "meas_q": [0.01, 1.58, 0.031],
            "meas_gripper": 46,
            "t": 0.1
        }
        
        # 4. Parse resposta
        assert ProtocolParser.is_measurement(response)
        meas = ProtocolParser.parse_measurement(response)
        
        # 5. Valida
        assert meas.is_valid()
        assert len(meas.q) == 3
    
    def test_all_commands_build_successfully(self):
        """Testa que todos os comandos podem ser construídos"""
        commands = [
            ProtocolBuilder.build_ping(),
            ProtocolBuilder.build_get_measurement(),
            ProtocolBuilder.build_set_reference([0, 0, 0], 0),
            ProtocolBuilder.build_set_gripper(0)
        ]
        
        for cmd in commands:
            assert isinstance(cmd, dict)
            assert "cmd" in cmd
            assert isinstance(cmd["cmd"], str)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])