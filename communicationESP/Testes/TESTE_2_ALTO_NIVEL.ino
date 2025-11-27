/*
 * ============================================================================
 * TESTE 2: CONTROLE ALTO NÍVEL (PC faz controle, ESP32 executa)
 * ============================================================================
 * 
 * O que faz:
 * - Recebe comandos de posição do PC (q_cmd)
 * - Simula motores com modelo simplificado
 * - Envia posição real medida de volta ao PC
 * 
 * Simula:
 * - Dinâmica de primeira ordem (tau = 100ms)
 * - Delay realista na resposta dos motores
 * - Perturbações (pode adicionar ruído)
 * 
 * Use para:
 * - Testar controlador sem hardware físico
 * - Validar algoritmos de controle
 * - Debug de problemas antes de usar motores reais
 * 
 * Conexões (quando usar hardware real):
 * - Motor 1: Potenciômetro no pino 32
 * - Motor 2: Potenciômetro no pino 33
 * - Motor 3: Potenciômetro no pino 34
 * - Garra:    Servo no pino 27
 * 
 * ============================================================================
 */

#include "Esp32Link.h"

// ===== CONFIGURAÇÃO WiFi =====
const char* SSID = "SUA_REDE";
const char* PASS = "SUA_SENHA";
const char* SERVER_IP = "192.168.1.100";
const uint16_t SERVER_PORT = 9000;
// =============================

// ===== CONFIGURAÇÃO SIMULAÇÃO =====
const float TAU = 0.1;           // Constante de tempo (100ms)
const float DT = 0.01;           // Período de amostragem (10ms)
const bool USE_REAL_HARDWARE = false;  // Mude para true se tiver motores
// ==================================

Esp32Link net;

RobotCommand    currentCommand;
RobotMeasurement currentMeasurement;

// Estado simulado
float q_real[3] = {0.0, 1.396, 0.03};  // Posição inicial
unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== CONTROLE ALTO NÍVEL ===");
  
  // Inicializa medição
  for (int i = 0; i < 3; i++) {
    currentMeasurement.q[i] = q_real[i];
    currentCommand.q_cmd[i] = q_real[i];
  }
  currentMeasurement.gripper = 0;
  currentCommand.gripper = 0;
  
  // Callbacks
  net.onConnected([]() {
    Serial.println("[✓] PC conectado!");
  });
  
  net.onDisconnected([]() {
    Serial.println("[✗] PC desconectado");
  });
  
  net.onSetReference([](const RobotCommand& cmd) {
    currentCommand = cmd;
    
    // Debug a cada 100 comandos
    static int count = 0;
    if (++count >= 100) {
      Serial.printf("[CMD] q: [%.3f, %.3f, %.3f]\n",
        cmd.q_cmd[0], cmd.q_cmd[1], cmd.q_cmd[2]);
      count = 0;
    }
  });
  
  // Conecta WiFi
  Serial.print("WiFi... ");
  if (!net.beginClient(SSID, PASS, SERVER_IP, SERVER_PORT)) {
    Serial.println("FALHA!");
    while(1) delay(1000);
  }
  Serial.println("OK!");
  Serial.println(net.localIP());
  
  lastUpdate = millis();
}

void loop() {
  // Atualiza posição simulada (modelo de 1ª ordem)
  unsigned long now = millis();
  if (now - lastUpdate >= (DT * 1000)) {
    lastUpdate = now;
    
    if (USE_REAL_HARDWARE) {
      // Leia sensores reais aqui
      // q_real[0] = lerPotenciometro(32);
      // q_real[1] = lerPotenciometro(33);
      // q_real[2] = lerPotenciometro(34);
    } else {
      // Simula dinâmica: q_real → q_cmd com constante de tempo TAU
      float alpha = DT / (TAU + DT);
      
      for (int i = 0; i < 3; i++) {
        q_real[i] += alpha * (currentCommand.q_cmd[i] - q_real[i]);
      }
    }
    
    // Atualiza medição
    for (int i = 0; i < 3; i++) {
      currentMeasurement.q[i] = q_real[i];
    }
    currentMeasurement.gripper = currentCommand.gripper;
  }
  
  net.setMeasurement(currentMeasurement);
  net.loop();
  
  delay(1);
}
