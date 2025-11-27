// Esp32Link.cpp
// Versão corrigida e compatível com protocol.py
#include "Esp32Link.h"
#include <ArduinoJson.h>

Esp32Link::Esp32Link()
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

bool Esp32Link::beginClient(const char* ssid, const char* pass, 
                             const char* serverIp, uint16_t serverPort) {
  _mode = Esp32LinkMode::NONE;

  // Conecta WiFi
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

void Esp32Link::loop() {
  if (_mode != Esp32LinkMode::STA_CLIENT) return;
  handleClient();
}

void Esp32Link::handleClient() {
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

void Esp32Link::readFrom(WiFiClient& c) {
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

void Esp32Link::parseLine(const String& line) {
  if (!line.startsWith("{")) return;

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

  // --- Comando "set_ref" ---
  if (cmd == "set_ref") {
    RobotCommand newCommand;
    
    // IMPORTANTE: Python pode enviar "q_cmd" OU "q_d"
    // Tentamos ambos para máxima compatibilidade
    JsonArray q_array;
    
    if (doc.containsKey("q_cmd")) {
      q_array = doc["q_cmd"];
    } else if (doc.containsKey("q_d")) {
      q_array = doc["q_d"];
    }

    if (q_array.size() == 3) {
      for (int i = 0; i < 3; i++) {
        newCommand.q_cmd[i] = q_array[i];
      }
      
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

  // --- Comando "get_meas" ---
  if (cmd == "get_meas") {
    _sendMeasurementResponse();
    return;
  }

  // --- Comando desconhecido ---
  sendLine("{\"ok\":false,\"err\":\"Comando desconhecido\"}");
}

void Esp32Link::_sendMeasurementResponse() {
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

void Esp32Link::setMeasurement(const RobotMeasurement& meas) {
  _measurementState = meas;
}

bool Esp32Link::sendLine(const String& line) {
  if (!_client.connected()) return false;
  
  String out = line;
  if (!out.endsWith("\n")) out += "\n";
  
  size_t n = _client.print(out);
  return n == out.length();
}

bool Esp32Link::isConnected() {
  return _client.connected();
}

IPAddress Esp32Link::localIP() const {
  return WiFi.localIP();
}

void Esp32Link::setConnected(bool now) {
  if (now && !_wasConnected) {
    _wasConnected = true;
    if (_onConnected) _onConnected();
  } else if (!now && _wasConnected) {
    _wasConnected = false;
    if (_onDisconnected) _onDisconnected();
  }
}
