// Esp32Link_PID.cpp
// Versão para controle PID de baixo nível
#include "Esp32Link_PID.h"
#include <ArduinoJson.h>

Esp32Link_PID::Esp32Link_PID()
: _mode(Esp32LinkMode::NONE),
  _serverHost(),
  _serverPort(0),
  _client(),
  _rxBuf(),
  _wasConnected(false),
  _lastConnectAttempt(0),
  _reconnectIntervalMs(1000),
  _measurementState(),
  _onConnected(nullptr),
  _onDisconnected(nullptr),
  _onSetReference(nullptr) {}

bool Esp32Link_PID::beginClient(const char* ssid, const char* pass, 
                                 const char* serverIp, uint16_t serverPort) {
  _mode = Esp32LinkMode::NONE;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(50);
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  _serverHost = serverIp;
  _serverPort = serverPort;
  _mode = Esp32LinkMode::STA_CLIENT;

  _rxBuf = "";
  _wasConnected = false;
  _lastConnectAttempt = 0;

  return true;
}

void Esp32Link_PID::loop() {
  if (_mode != Esp32LinkMode::STA_CLIENT) return;
  handleClient();
}

void Esp32Link_PID::handleClient() {
  if (!_client.connected()) {
    setConnected(false);
    
    uint32_t now = millis();
    if (now - _lastConnectAttempt >= _reconnectIntervalMs) {
      _lastConnectAttempt = now;
      _client.stop();
      _client.connect(_serverHost.c_str(), _serverPort);
    }
  } else {
    setConnected(true);
    
    if (_client.available()) {
      readFrom(_client);
    }
  }
}

void Esp32Link_PID::readFrom(WiFiClient& c) {
  while (c.available()) {
    char ch = static_cast<char>(c.read());
    
    if (ch == '\r') continue;
    
    if (ch == '\n') {
      String line = _rxBuf;
      _rxBuf = "";
      line.trim();
      
      if (line.length() > 0) {
        parseLine(line);
      }
    } else {
      if (_rxBuf.length() < 1024) {
        _rxBuf += ch;
      } else {
        _rxBuf = "";
      }
    }
  }
}

void Esp32Link_PID::parseLine(const String& line) {
  if (!line.startsWith("{")) return;

  // ===== DEBUG: Mostra JSON cru =====
  Serial.println("[DEBUG ESP] JSON recebido:");
  Serial.println(line);
  Serial.println("---");
  // ==================================

  StaticJsonDocument<512> doc;
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

  // --- Comando "set_ref" (trajetória completa) ---
  if (cmd == "set_ref") {
    RobotCommand newCommand;
    
    // Aceita tanto "q_cmd" (compatibilidade) quanto "q_d" (padrão PID)
    JsonArray q_array;
    if (doc.containsKey("q_d")) {
      q_array = doc["q_d"];
    } else if (doc.containsKey("q_cmd")) {
      q_array = doc["q_cmd"];
    }

    if (q_array.size() == 3) {
      // Posição
      for (int i = 0; i < 3; i++) {
        newCommand.q_d[i] = q_array[i];
      }
      
      // Velocidade (opcional)
      if (doc.containsKey("qd_d")) {
        JsonArray qd_array = doc["qd_d"];
        if (qd_array.size() == 3) {
          for (int i = 0; i < 3; i++) {
            newCommand.qd_d[i] = qd_array[i];
          }
        }
      }
      
      // Aceleração (opcional)
      if (doc.containsKey("qdd_d")) {
        JsonArray qdd_array = doc["qdd_d"];
        if (qdd_array.size() == 3) {
          for (int i = 0; i < 3; i++) {
            newCommand.qdd_d[i] = qdd_array[i];
          }
        }
      }
      
      // Garra
      if (doc.containsKey("gripper")) {
        newCommand.gripper = doc["gripper"];
      }
      
      if (_onSetReference) {
        _onSetReference(newCommand);
      }
      
      sendLine("{\"ok\":true}");
    } else {
      // Talvez só gripper
      if (doc.containsKey("gripper")) {
        newCommand.gripper = doc["gripper"];
        if (_onSetReference) {
          _onSetReference(newCommand);
        }
        sendLine("{\"ok\":true}");
      } else {
        sendLine("{\"ok\":false,\"err\":\"set_ref incompleto\"}");
      }
    }
    return;
  }

  // --- Comando "set_gripper" ---
  if (cmd == "set_gripper") {
    RobotCommand newCommand;
    newCommand.gripper = doc["value"];
    
    if (_onSetReference) {
      _onSetReference(newCommand);
    }
    
    sendLine("{\"ok\":true}");
    return;
  }

  // --- Comando "set_gains" ---
  if (cmd == "set_gains") {
    RobotCommand newCommand;

    // Aceita AMBOS (compatibilidade máxima)
    newCommand.kp = doc["Kp"];  // Maiúsculo
    newCommand.kd = doc["Kd"];
    newCommand.ki = doc["Ki"];


    // ===== DEBUG: Valores do JSON =====
    Serial.println("[DEBUG ESP] Parsing set_gains:");
    Serial.printf("  doc[\"kp\"] = %.4f\n", (float)doc["kp"]);
    Serial.printf("  doc[\"kd\"] = %.4f\n", (float)doc["kd"]);
    Serial.printf("  doc[\"ki\"] = %.4f\n", (float)doc["ki"]);
    // ==================================
    
    // IMPORTANTE: Usar lowercase (kp, kd, ki)
    newCommand.kp = doc["kp"];
    newCommand.kd = doc["kd"];
    newCommand.ki = doc["ki"];
    
    if (_onSetReference) {
      _onSetReference(newCommand);
    }
    
    sendLine("{\"ok\":true}");
    return;
  }

  // --- Comando "get_meas" ---
  if (cmd == "get_meas") {
    _sendMeasurementResponse();
    return;
  }

  // --- Comando desconhecido ---
  sendLine("{\"ok\":false,\"err\":\"Comando desconhecido\"}");
}

void Esp32Link_PID::_sendMeasurementResponse() {
  StaticJsonDocument<256> doc;

  doc["t"] = millis();

  JsonArray q_array = doc.createNestedArray("meas_q");
  for (int i = 0; i < 3; i++) {
    q_array.add(_measurementState.q[i]);
  }
  
  doc["meas_gripper"] = _measurementState.gripper;

  String output;
  serializeJson(doc, output);
  sendLine(output);
}

void Esp32Link_PID::setMeasurement(const RobotMeasurement& meas) {
  _measurementState = meas;
}

bool Esp32Link_PID::sendLine(const String& line) {
  if (!_client.connected()) return false;
  
  String out = line;
  if (!out.endsWith("\n")) out += "\n";
  
  size_t n = _client.print(out);
  return n == out.length();
}

bool Esp32Link_PID::isConnected() {
  return _client.connected();
}

IPAddress Esp32Link_PID::localIP() const {
  return WiFi.localIP();
}

void Esp32Link_PID::setConnected(bool now) {
  if (now && !_wasConnected) {
    _wasConnected = true;
    if (_onConnected) _onConnected();
  } else if (!now && _wasConnected) {
    _wasConnected = false;
    if (_onDisconnected) _onDisconnected();
  }
}
