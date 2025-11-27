// Esp32Link_PID.h
// Versão para controle PID de baixo nível (ESP32 faz controle)
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <functional>

enum class Esp32LinkMode : uint8_t { NONE, STA_CLIENT };

/**
 * @struct RobotCommand
 * @brief Comando com trajetória completa + ganhos PID
 */
struct RobotCommand {
  // Trajetória desejada
  float q_d[3]   = {0.0f, 0.0f, 0.0f};  // Posição
  float qd_d[3]  = {0.0f, 0.0f, 0.0f};  // Velocidade (feedforward)
  float qdd_d[3] = {0.0f, 0.0f, 0.0f};  // Aceleração (feedforward)
  
  // Garra
  int gripper = 0;
  
  // Ganhos PID (podem ser atualizados via set_gains)
  float kp = 0.0f;
  float kd = 0.0f;
  float ki = 0.0f;
};

/**
 * @struct RobotMeasurement
 * @brief Medição que a ESP32 envia para o PC
 */
struct RobotMeasurement {
  float q[3]    = {0.0f, 0.0f, 0.0f}; // Posição real medida
  int   gripper = 0;                   // Posição real da garra
};

// Tipos de Callback
using VoidCallback    = std::function<void(void)>;
using CommandCallback = std::function<void(const RobotCommand& command)>;

class Esp32Link_PID {
public:
  Esp32Link_PID();

  // Inicialização
  bool beginClient(const char* ssid, const char* pass, 
                   const char* serverIp, uint16_t serverPort);
  
  // Loop principal
  void loop();

  // Atualizar medição que será enviada ao PC
  void setMeasurement(const RobotMeasurement& meas);

  // Estado
  bool        isConnected();
  IPAddress   localIP() const;
  const char* modeName() const { return "STA_CLIENT"; }

  // Callbacks
  void onConnected(VoidCallback cb)       { _onConnected = cb; }
  void onDisconnected(VoidCallback cb)    { _onDisconnected = cb; }
  void onSetReference(CommandCallback cb) { _onSetReference = cb; }

private:
  void handleClient();
  void readFrom(WiFiClient& c);
  void parseLine(const String& line);
  void setConnected(bool now);
  bool sendLine(const String& line);
  void _sendMeasurementResponse();

  // Estado de conexão
  Esp32LinkMode _mode;
  String        _serverHost;
  uint16_t      _serverPort;
  WiFiClient    _client;
  String        _rxBuf;
  bool          _wasConnected;
  uint32_t      _lastConnectAttempt;
  uint32_t      _reconnectIntervalMs;

  // Estado da medição
  RobotMeasurement _measurementState;

  // Callbacks
  VoidCallback    _onConnected;
  VoidCallback    _onDisconnected;
  CommandCallback _onSetReference;
};
