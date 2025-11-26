// Esp32Link_PID.cpp
#include "Esp32Link_PID.h" 
#include <ArduinoJson.h>     // Necessário instalar essa biblioteca no Arduino Ide

// Construtor (função de inicialização) da classe.
// Apenas zera as variáveis iniciais.
Esp32Link_PID::Esp32Link_PID() 
: _mode(Esp32LinkMode::NONE), // Modo inicial
  _serverHost(),
  _serverPort(0),
  _client(),
  _rxBuf(),
  _wasConnected(false),
  _lastConnectAttempt(0),
  _reconnectIntervalMs(1000), // Tenta reconectar a cada 1 segundo
  _measurementState(),        // Zera a struct de medição
  _onConnected(nullptr),
  _onDisconnected(nullptr),
  _onSetReference(nullptr) {} // Callback para quando um comando chegar

//Inicia a conexão WiFi e prepara para conectar ao PC.
bool Esp32Link_PID::beginClient(const char* ssid, const char* pass, const char* serverIp, uint16_t serverPort) {
  _mode = Esp32LinkMode::NONE;

  // Conecta no Wi-Fi (Modo Estação)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  
  uint32_t t0 = millis();
  // Espera até 15 segundos para conectar
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(50); // Dá um tempinho para o WiFi trabalhar
  }
  
  // Se depois de 15s não conectou, retorna erro.
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  // Guarda o IP e a Porta do servidor (PC)
  _serverHost = serverIp;
  _serverPort = serverPort;
  _mode = Esp32LinkMode::STA_CLIENT;

  // Reseta o estado da conexão
  _rxBuf = "";
  _wasConnected = false;
  _lastConnectAttempt = 0;

  return true;
}

// Função principal.
// Ela cuida de manter a conexão e ler os dados que chegam.
void Esp32Link_PID::loop() {
  if (_mode != Esp32LinkMode::STA_CLIENT) {
    return; // Se não está no modo cliente, não faz nada.
  }
  // cuida de conectar, reconectar e ler dados
  handleClient();
}

//Gerencia a conexão com o servidor (PC).
void Esp32Link_PID::handleClient() {
  // Se não está conectado tenta conectar
  if (!_client.connected()) {
    setConnected(false); // Estado de não conexão
    
    // Tenta reconectar, mas só a cada 1 segundo
    uint32_t now = millis();
    if (now - _lastConnectAttempt >= _reconnectIntervalMs) {
      _lastConnectAttempt = now;
      _client.stop(); // Garante que a conexão antiga foi fechada
      _client.connect(_serverHost.c_str(), _serverPort);
    }
  } 
  else {
    setConnected(true); // Estado de conexão
    
    // Verifica se o PC enviou algum dado
    if (_client.available()) {
      readFrom(_client);
    }
  }
}

// Lê os dados que chegam do PC, caractere por caractere.
void Esp32Link_PID::readFrom(WiFiClient& c) {
  while (c.available()) {
    char ch = static_cast<char>(c.read());
    
    if (ch == '\r') continue; // Ignora o caractere '\r'
    
    // Se for o fim da linha ('\n')
    if (ch == '\n') {
      String line = _rxBuf; // Copia os dados para uma linha
      _rxBuf = "";          // Limpa o buffer para a próxima
      
      line.trim(); // Remove espaços em branco do início e fim
      
      if (line.length() > 0) {
        parseLine(line); // Envia a linha para ser "traduzida"
      }
    } 
    else {
      // Adiciona no buffer, mas limita o tamanho (segurança)
      if (_rxBuf.length() < 1024) {
        _rxBuf += ch;
      } else {
        _rxBuf = ""; // Overflow, limpa tudo
      }
    }
  }
}

// "Traduz" a linha de JSON que veio do PC e decide o que fazer.
void Esp32Link_PID::parseLine(const String& line) {

  // (Debug: imprime a linha que chegou)
  // Serial.println("[ESP] Linha Bruta Recebida:");
  // Serial.println(line);

  if (!line.startsWith("{")) {
    return;
  }

  // Prepara um "documento" JSON para ler os dados.
  StaticJsonDocument<512> doc; // Usamos 512 por segurança
  
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    sendLine("{\"ok\":false,\"err\":\"JSON mal formatado\"}");
    return;
  }

  String cmd = doc["cmd"].as<String>();

  // --- Comando "ping" ---
  if (cmd == "ping") {
    sendLine("{\"pong\":true}"); 
    return;
  }

  // --- Comando "set_ref" (Enviado PELA SIMULAÇÃO) ---
  if (cmd == "set_ref") {
    
    RobotCommand newCommand; 
    
    // Procura pela trajetória desejada que o Python (V7) envia
    JsonArray q_d_array = doc["q_d"];
    JsonArray qd_d_array = doc["qd_d"];
    JsonArray qdd_d_array = doc["qdd_d"];

    // Verifica se a trajetória principal chegou
    if (q_d_array.size() == 3) {
      
      // Copia a trajetória (q, qd, qdd)
      for (int i = 0; i < 3; i++) {
        newCommand.q_d[i] = q_d_array[i];
        // Checa se qd e qdd também vieram (eles podem ser opcionais)
        if (qd_d_array.size() == 3) newCommand.qd_d[i] = qd_d_array[i];
        if (qdd_d_array.size() == 3) newCommand.qdd_d[i] = qdd_d_array[i];
      }
      
      // Copia o comando da garra (se ele veio junto)
      if (doc.containsKey("gripper")) {
          newCommand.gripper = doc["gripper"]; 
      }

      if (doc.containsKey("kp")) newCommand.kp = doc["kp"];
      if (doc.containsKey("kd")) newCommand.kd = doc["kd"];
      if (doc.containsKey("ki")) newCommand.ki = doc["ki"];
      
      // Envia o pacote de comando para o callback (para o .ino)
      if (_onSetReference) {
        _onSetReference(newCommand);
      }
      
      sendLine("{\"ok\":true}"); // Avisa o PC que deu certo
    
    } else {
      // O PC não enviou 'q_d', talvez só 'gripper'? (Ex: no Sync antigo)
      if (doc.containsKey("gripper")) {
          newCommand.gripper = doc["gripper"];
          if (_onSetReference) {
              _onSetReference(newCommand);
          }
          sendLine("{\"ok\":true}");
      } else {
          // Pacote 'set_ref' inválido
          sendLine("{\"ok\":false,\"err\":\"Comando 'set_ref' incompleto\"}");
      }
    }
    return;
  }

  // --- Comando "set_gripper" (Enviado pelo SLIDER) ---
  if (cmd == "set_gripper") {
      RobotCommand newCommand;
      newCommand.gripper = doc["value"];
      
      if (_onSetReference) {
        _onSetReference(newCommand);
      }
      
      Serial.printf("[ESP] Comando garra (direto): %d\n", newCommand.gripper);
      sendLine("{\"ok\":true}");
      return;
  }

  if (cmd == "set_gains") {
      RobotCommand newCommand;
      
      // Lê os ganhos do JSON
      newCommand.kp = doc["kp"];
      newCommand.kd = doc["kd"];
      newCommand.ki = doc["ki"];
      
      // (Opcional: Avisa no serial para debug)
      Serial.printf("[ESP] Ganhos recebidos (direto): P:%.3f D:%.3f I:%.3f\n", newCommand.kp, newCommand.kd, newCommand.ki);

      // Chama o callback para atualizar o .ino
      if (_onSetReference) {
        _onSetReference(newCommand);
      }
      
      sendLine("{\"ok\":true}");
      return;
  }

  // --- Comando "get_meas" (PC pede os dados) ---
  if (cmd == "get_meas") {
    _sendMeasurementResponse(); // Chama a função que envia
    return; 
  }

  // --- Fallback (Comando desconhecido) ---
  if (cmd.length() > 0) {
     sendLine("{\"ok\":false,\"err\":\"Comando desconhecido\"}");
     return;
  }
  
  sendLine("{\"ok\":false,\"err\":\"JSON desconhecido\"}");
}

// Monta e envia a resposta para "get_meas".
void Esp32Link_PID::_sendMeasurementResponse() {
  // <<< MODIFICADO (V7): JSON de resposta muito mais simples >>>
  StaticJsonDocument<256> doc; 

  doc["t"] = millis(); // Adiciona o tempo atual

  // Cria o array de posição medida
  JsonArray q_array = doc.createNestedArray("meas_q");
  
  // Copia os valores da nossa 'struct' interna para o JSON
  for (int i = 0; i < 3; i++) {
    q_array.add(_measurementState.q[i]);
  }
  
  // Adiciona o valor da garra (lido)
  doc["meas_gripper"] = _measurementState.gripper;

  // <<< REMOVIDO: Não enviamos mais qdd, qdd2, ou i >>>

  // Converte o JSON para String
  String output;
  serializeJson(doc, output);

  // Envia o texto pela rede
  sendLine(output);
}

// Atualiza os valores de medição (chamado pelo .ino)
void Esp32Link_PID::setMeasurement(const RobotMeasurement& meas) {
  // Copia os valores do .ino para a variável interna da biblioteca
  _measurementState = meas;
}

// Envia uma linha de String para o PC.
bool Esp32Link_PID::sendLine(const String& line) {
  if (!_client.connected()) {
    return false;
  }
  String out = line;
  if (!out.endsWith("\n")) {
    out += "\n";
  }
  size_t n = _client.print(out);
  return n == out.length(); // Retorna true se conseguiu enviar tudo
}

// Retorna true se o cliente TCP estiver conectado ao servidor.
bool Esp32Link_PID::isConnected() {
  return _client.connected();
}

// Retorna o IP local da ESP32 na rede WiFi.
IPAddress Esp32Link_PID::localIP() const {
  return WiFi.localIP();
}

// Controla os callbacks _onConnected e _onDisconnected.
void Esp32Link_PID::setConnected(bool now) {
  // Se está conectado agora (e antes estáva desconectados)
  if (now && !_wasConnected) {
    _wasConnected = true;
    if (_onConnected) {
      _onConnected(); // Chama o callback
    }
  } 
  // Se está desconectado agora (e antes estáva conectados)
  else if (!now && _wasConnected) {
    _wasConnected = false;
    if (_onDisconnected) {
      _onDisconnected(); // Chama o callback
    }
  }
}