// Esp32Link_PID.h
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <functional>

// O modo de operação (STA_CLIENT é o único que usamos)
enum class Esp32LinkMode : uint8_t { NONE, STA_CLIENT };

/**
 * @struct RobotCommand
 * @brief Pacote de dados que o PC (Host) envia para a ESP32.
 * Contém a trajetória desejada (q, qd, qdd) e o comando da garra.
 */
struct RobotCommand {
  // <<< MODIFICADO (V7): Adicionamos a trajetória completa >>>
  float q_d[3]   = {0.0f, 0.0f, 0.0f};   // Posição Desejada
  float qd_d[3]  = {0.0f, 0.0f, 0.0f};  // Velocidade Desejada (para Feedforward)
  float qdd_d[3] = {0.0f, 0.0f, 0.0f}; // Aceleração Desejada (para Feedforward)
  int   gripper  = 0;  
  float kp = 0.0f;
  float kd = 0.0f;
  float ki = 0.0f;
};

/**
 * @struct RobotMeasurement
 * @brief Pacote de dados que a ESP32 (Client) envia para o PC.
 * (Esta struct não muda, já está correta)
 */
struct RobotMeasurement {
  float q[3]   = {0.0f, 0.0f, 0.0f}; // Posição real medida
  int   gripper = 0;                // Posição real da garra
};


// --- Tipos de Callback ---
using VoidCallback    = std::function<void(void)>;
// (Este callback já usa a 'RobotCommand', então está correto)
using CommandCallback = std::function<void(const RobotCommand& command)>; 


// <<< MODIFICADO (V7): O nome da classe mudou >>>
class Esp32Link_PID {
public:
  // <<< MODIFICADO (V7): Nome do construtor >>>
  Esp32Link_PID();

  bool beginClient(const char* ssid, const char* pass, const char* serverIp, uint16_t serverPort);
  void loop();

  /**
   * @brief Atualiza o estado da medição que será enviada ao PC.
   */
  void setMeasurement(const RobotMeasurement& meas);

  // ---------- Estado/Utilidades ----------
  bool        isConnected();
  IPAddress   localIP() const;
  const char* modeName() const { return "STA_CLIENT"; }

  // ---------- Callbacks (Eventos) ----------
  void onConnected(VoidCallback cb)    { _onConnected = cb; }
  void onDisconnected(VoidCallback cb) { _onDisconnected = cb; }

  /**
   * @brief Evento chamado quando o PC envia um comando ('set_ref' ou 'set_gripper').
   */
  void onSetReference(CommandCallback cb) { _onSetReference = cb; }

private:
  // ---- Funções Internas ----
  void handleClient();
  void readFrom(WiFiClient& c);
  void parseLine(const String& line); 
  void setConnected(bool now);
  bool sendLine(const String& line);
  void _sendMeasurementResponse();

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
  RobotMeasurement _measurementState; 

  // ---- Callbacks ----
  VoidCallback    _onConnected;
  VoidCallback    _onDisconnected;
  CommandCallback _onSetReference;
};