# Testes do Sistema CocoaBot

Este diretório contém testes automatizados para validar o sistema de comunicação e controle do robô CocoaBot.

## 📦 Estrutura

```
tests/
├── __init__.py                 # Inicialização do pacote
├── conftest.py                 # Fixtures compartilhadas (pytest)
├── test_protocol.py            # Testes do protocolo ⭐ COMECE AQUI
├── test_comm_manager.py        # Testes do gerenciador
├── test_esp32_server.py        # Testes do servidor TCP
├── test_simulation_thread.py   # Testes da thread de simulação
├── test_integration.py         # Testes de integração
├── mock_esp32.py               # Simulador de ESP32 ⭐ ÚTIL
└── README.md                   # Este arquivo
```

## 🚀 Quick Start

### 1. Instalação de Dependências

```bash
# Instalar pytest e plugins
pip install pytest pytest-qt pytest-mock pytest-cov

# Ou use requirements_test.txt (se disponível)
pip install -r requirements_test.txt
```

### 2. Rodar Todos os Testes

```bash
# Do diretório raiz do projeto
pytest tests/

# Ou com mais detalhes
pytest tests/ -v

# Com cobertura de código
pytest tests/ --cov=communication --cov-report=html
```

### 3. Rodar Teste Específico

```bash
# Apenas testes do protocolo (rápidos, sem mocks)
pytest tests/test_protocol.py

# Apenas testes do gerenciador
pytest tests/test_comm_manager.py

# Teste específico
pytest tests/test_protocol.py::TestProtocolBuilder::test_build_ping
```

## 📋 Descrição dos Testes

### ⭐ test_protocol.py (COMECE AQUI)
**O que testa:** Builders, parsers e validação do protocolo  
**Dependências:** Nenhuma (nem Qt, nem rede)  
**Velocidade:** Muito rápido (~0.1s)  
**Cobertura:** 100% do módulo `protocol.py`

```bash
pytest tests/test_protocol.py -v
```

**Classes testadas:**
- `ESP32Commands` e `ESP32Responses` (constantes)
- `ProtocolBuilder` (construção de comandos)
- `ProtocolParser` (interpretação de respostas)
- `MeasurementData` (validação de dados)

**Por que começar aqui:**
- Testes simples e rápidos
- Valida base do protocolo
- Não requer hardware ou mocks complexos
- Ajuda a entender o protocolo

---

### test_comm_manager.py
**O que testa:** API de alto nível do `CommunicationManager`  
**Dependências:** PyQt6, mocks do servidor  
**Velocidade:** Rápido (~1s)  
**Cobertura:** ~80% do `comm_manager.py`

```bash
pytest tests/test_comm_manager.py -v
```

**Testa:**
- Inicialização do gerenciador
- Envio de comandos (ping, get_meas, set_ref, set_gripper)
- Recepção e processamento de respostas
- Sincronização de posição inicial
- Emissão de sinais Qt

**Usa mocks para:**
- `ESP32Server` (não precisa de TCP real)
- Respostas da ESP32 (controladas)

---

### test_esp32_server.py (TODO)
**O que testa:** Servidor TCP puro  
**Dependências:** PyQt6, mocks de socket  
**Velocidade:** Médio (~2s)  

**Testará:**
- Inicialização do servidor
- Conexão/desconexão de clientes
- Envio/recepção de JSON
- Parser de linhas (NDJSON)
- Tratamento de erros

---

### test_simulation_thread.py (TODO)
**O que testa:** Thread de simulação  
**Dependências:** PyQt6, mock do CommunicationManager  
**Velocidade:** Médio (~2s)  

**Testará:**
- Configuração da thread
- Loop de simulação
- Integração com controlador
- Sincronização inicial
- Envio de comandos durante simulação

---

### test_integration.py (TODO)
**O que testa:** Fluxos completos end-to-end  
**Dependências:** Todos os componentes  
**Velocidade:** Lento (~5-10s)  

**Testará:**
- Servidor real + MockESP32
- Fluxo completo de comunicação
- Sincronização real
- Simulação completa (curta)

---

## 🤖 Simulador MockESP32

O arquivo `mock_esp32.py` fornece um simulador de ESP32 que pode ser usado:

### Como Módulo em Testes

```python
from tests.mock_esp32 import MockESP32, create_mock_esp32_for_testing

# Criar mock simples
mock = create_mock_esp32_for_testing(initial_q=[0, 0, 0])

# Processar comando
response = mock.process_command({"cmd": "get_meas"})
print(response)  # {"meas_q": [...], "meas_gripper": 0, "t": 0.0}
```

### Como Processo Standalone

```bash
# Terminal 1: Inicia mock ESP32
python tests/mock_esp32.py --port 9000

# Terminal 2: Inicia aplicação principal
python main.py
```

**Opções:**
```bash
python tests/mock_esp32.py --help

Argumentos:
  --host HOST    Host do servidor (padrão: localhost)
  --port PORT    Porta do servidor (padrão: 9000)
  --noise FLOAT  Nível de ruído nas medições (padrão: 0.001)
  --delay INT    Delay de resposta em ms (padrão: 10)
```

**Exemplo com configurações:**
```bash
# ESP32 com mais ruído e delay
python tests/mock_esp32.py --noise 0.01 --delay 50
```

---

## 📊 Cobertura de Código

### Gerar Relatório de Cobertura

```bash
# Rodar testes com cobertura
pytest tests/ --cov=communication --cov-report=html

# Abrir relatório no navegador
# O relatório estará em htmlcov/index.html
```

### Ver Cobertura no Terminal

```bash
pytest tests/ --cov=communication --cov-report=term-missing
```

**Output esperado:**
```
Name                              Stmts   Miss  Cover   Missing
---------------------------------------------------------------
communication/__init__.py             7      0   100%
communication/protocol.py            85      5    94%   45-47
communication/esp32_server.py       120     25    79%   78-85, 102-110
communication/comm_manager.py       150     30    80%   145-160, 180-195
---------------------------------------------------------------
TOTAL                               362     60    83%
```

---

## 🎯 Objetivos de Cobertura

| Módulo | Cobertura Alvo | Cobertura Atual | Status |
|--------|----------------|-----------------|--------|
| protocol.py | 100% | 100% | ✅ |
| comm_manager.py | 80% | 80% | ✅ |
| esp32_server.py | 70% | 0% | ⏳ TODO |
| simulation_thread.py | 60% | 0% | ⏳ TODO |

---

## ⚠️ Problemas Comuns

### Erro: "No module named 'communication'"

**Solução:** Rode testes do diretório raiz do projeto
```bash
cd /caminho/para/projeto
pytest tests/
```

### Erro: "pytest: command not found"

**Solução:** Instale pytest
```bash
pip install pytest pytest-qt pytest-mock
```

### Erro: "QApplication not found"

**Solução:** Instale PyQt6
```bash
pip install PyQt6
```

### Testes de Qt Falham

**Solução:** Use plugin pytest-qt
```bash
pip install pytest-qt

# Use fixture qtbot nos testes
def test_something(qtbot):
    manager = CommunicationManager()
    qtbot.addWidget(manager)  # Registra para cleanup
```

### Mock ESP32 Não Conecta

**Soluções:**
1. Certifique-se que o servidor está rodando primeiro
2. Verifique a porta (padrão: 9000)
3. Verifique firewall
4. Use `--host localhost` explicitamente

---

## 🔧 Configuração Avançada

### pytest.ini (Opcional)

Crie na raiz do projeto:

```ini
[pytest]
testpaths = tests
python_files = test_*.py
python_classes = Test*
python_functions = test_*
addopts = 
    -v
    --strict-markers
    --tb=short
markers =
    slow: marks tests as slow (deselect with '-m "not slow"')
    integration: marks tests as integration tests
```

### requirements_test.txt (Opcional)

```txt
pytest>=7.0.0
pytest-qt>=4.0.0
pytest-mock>=3.10.0
pytest-cov>=4.0.0
PyQt6>=6.4.0
numpy>=1.20.0
```

---

## 📝 Boas Práticas

### 1. Rode Testes Antes de Commit
```bash
# Testes rápidos
pytest tests/test_protocol.py

# Todos os testes
pytest tests/
```

### 2. Escreva Testes para Novos Recursos
Quando adicionar um novo comando ao protocolo:
```python
# Em test_protocol.py
def test_build_new_command(self):
    cmd = ProtocolBuilder.build_new_command(param=value)
    assert cmd["cmd"] == "new_command"
    assert cmd["param"] == value
```

### 3. Use Fixtures para Dados Comuns
```python
# Em conftest.py
@pytest.fixture
def my_test_data():
    return {"key": "value"}

# Em test_*.py
def test_something(my_test_data):
    assert my_test_data["key"] == "value"
```

### 4. Organize Testes em Classes
```python
class TestFeatureX:
    """Testes relacionados ao Feature X"""
    
    def test_case_1(self):
        ...
    
    def test_case_2(self):
        ...
```

---

## 🚀 Próximos Passos

### Curto Prazo
- [ ] Implementar `test_esp32_server.py`
- [ ] Implementar `test_simulation_thread.py`
- [ ] Aumentar cobertura de `comm_manager.py` para 90%

### Médio Prazo
- [ ] Implementar `test_integration.py`
- [ ] Adicionar testes de performance
- [ ] Configurar CI/CD (GitHub Actions)

### Longo Prazo
- [ ] Testes de stress (muitas conexões)
- [ ] Testes de concorrência
- [ ] Testes de recovery (falhas de rede)

---

## 📚 Referências

- [pytest Documentation](https://docs.pytest.org/)
- [pytest-qt Documentation](https://pytest-qt.readthedocs.io/)
- [Python unittest.mock](https://docs.python.org/3/library/unittest.mock.html)
- [PyQt6 Documentation](https://www.riverbankcomputing.com/static/Docs/PyQt6/)

---

## 💡 Dicas

### Rodar Apenas Testes Rápidos
```bash
pytest tests/test_protocol.py -v
```

### Debug de Teste Específico
```bash
pytest tests/test_protocol.py::TestProtocolBuilder::test_build_ping -vv --tb=long
```

### Ver Print Statements
```bash
pytest tests/test_protocol.py -s
```

### Rodar em Paralelo (mais rápido)
```bash
pip install pytest-xdist
pytest tests/ -n auto
```

---

## 📞 Suporte

Se encontrar problemas:
1. Verifique que todas as dependências estão instaladas
2. Rode testes do diretório raiz do projeto
3. Consulte a seção "Problemas Comuns"
4. Abra uma issue no repositório

---

**Última atualização:** Novembro 2025  
**Versão dos testes:** 1.0.0