// Esp32Link.h
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <functional>

// O modo de operação (STA_CLIENT é o único que usamos)
enum class Esp32LinkMode : uint8_t { NONE, STA_CLIENT };

/**
 * RobotCommand
 * Pacote de dados que o PC (Host) envia para a ESP32.
 * Contém o comando de posição e o comando da garra.
 */
struct RobotCommand {
  float q_cmd[3] = {0.0f, 0.0f, 0.0f}; // Posição comandada
  int   gripper  = 0;                 // Posição da garra
};

/**
 * RobotMeasurement
 * Pacote de dados que a ESP32 (Client) envia para o PC.
 * Contém a posição real lida e a posição real da garra.
 */
struct RobotMeasurement {
  float q[3]   = {0.0f, 0.0f, 0.0f}; // Posição real medida
  int   gripper = 0;                // Posição real da garra
};


// --- Tipos de Callback ---
using VoidCallback    = std::function<void(void)>;
// <<< MODIFICADO: Callback agora usa o struct 'RobotCommand' >>>
using CommandCallback = std::function<void(const RobotCommand& command)>; 


class Esp32Link {
public:
  Esp32Link();

  bool beginClient(const char* ssid, const char* pass, const char* serverIp, uint16_t serverPort);
  void loop();

  /**
   *  Atualiza o estado da medição que será enviada ao PC.
   *  Chame isso no seu loop() com os dados dos seus sensores.
   *  meas A struct RobotMeasurement com os valores de q e gripper.
   */
  void setMeasurement(const RobotMeasurement& meas);

  // ---------- Estado/Utilidades ----------
  bool        isConnected();
  IPAddress   localIP() const;
  const char* modeName() const { return "STA_CLIENT"; }

  // ---------- Callbacks (Eventos) ----------
  void onConnected(VoidCallback cb)    { _onConnected = cb; }
  void onDisconnected(VoidCallback cb) { _onDisconnected = cb; }

  // Evento chamado quando o PC envia um comando ('set_ref' ou 'set_gripper').
  void onSetReference(CommandCallback cb) { _onSetReference = cb; }

private:
  // ---- Funções Internas ----
  void handleClient();
  void readFrom(WiFiClient& c);
  void parseLine(const String& line); 
  void setConnected(bool now);
  bool sendLine(const String& line);
  void _sendMeasurementResponse(); // (Função que envia a medição)

  // ---- Estado de conexão ----
  Esp32LinkMode _mode;
  String        _serverHost;
  uint16_t      _serverPort;
  WiFiClient    _client;
  String        _rxBuf;
  bool          _wasConnected;
  uint32_t      _lastConnectAttempt;
  uint32_t      _reconnectIntervalMs;

  // ---- Estado da Medição ----
  // <<< MODIFICADO: Armazena a nova struct 'RobotMeasurement' >>>
  RobotMeasurement _measurementState; 

  // ---- Callbacks ----
  VoidCallback    _onConnected;
  VoidCallback    _onDisconnected;
  CommandCallback _onSetReference;
};