/*
 * ============================================================================
 * TESTE 1: ECO/BATE-VOLTA (Teste de Comunicação)
 * ============================================================================
 * 
 * O que faz:
 * - Conecta no WiFi e no servidor PC
 * - Recebe comandos de posição e garra
 * - Devolve os mesmos valores como "medição"
 * - Imprime no Serial para debug
 * 
 * Use para:
 * - Verificar se a comunicação está funcionando
 * - Testar o protocolo de mensagens
 * - Validar compatibilidade Python <-> ESP32
 * 
 * Resultado esperado:
 * - No Python, você verá q_real = q_comando
 * - Gráficos devem seguir perfeitamente a trajetória
 * 
 * ============================================================================
 */

#include "Esp32Link.h"  // Biblioteca básica

// ===== CONFIGURAÇÃO WiFi =====
const char* SSID = "SUA_REDE";          // <-- ALTERE AQUI
const char* PASS = "SUA_SENHA";         // <-- ALTERE AQUI
const char* SERVER_IP = "192.168.1.100"; // <-- IP do seu PC
const uint16_t SERVER_PORT = 9000;
// ===============================

Esp32Link net;

RobotCommand    currentCommand;
RobotMeasurement currentMeasurement;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== TESTE DE COMUNICAÇÃO: ECO ===");
  
  // Posição inicial padrão
  currentCommand.q_cmd[0] = 0.0;       // q1 = 0 rad
  currentCommand.q_cmd[1] = 1.396;     // q2 = 80° em rad
  currentCommand.q_cmd[2] = 0.03;      // d3 = 3 cm
  currentCommand.gripper = 0;
  
  // Inicializa medição
  for (int i = 0; i < 3; i++) {
    currentMeasurement.q[i] = currentCommand.q_cmd[i];
  }
  currentMeasurement.gripper = currentCommand.gripper;
  
  // Callbacks
  net.onConnected([]() {
    Serial.println("[✓] Conectado ao PC!");
  });
  
  net.onDisconnected([]() {
    Serial.println("[✗] Desconectado do PC");
  });
  
  net.onSetReference([](const RobotCommand& cmd) {
    currentCommand = cmd;
    
    // Debug: imprime comando recebido
    Serial.printf("[RX] q: [%.3f, %.3f, %.3f] | garra: %d°\n",
      cmd.q_cmd[0], cmd.q_cmd[1], cmd.q_cmd[2], cmd.gripper);
  });
  
  // Conecta WiFi e servidor
  Serial.print("Conectando WiFi... ");
  if (!net.beginClient(SSID, PASS, SERVER_IP, SERVER_PORT)) {
    Serial.println("FALHA!");
    Serial.println("Verifique: SSID, PASS, IP do PC");
    while(1) delay(1000);
  }
  
  Serial.println("OK!");
  Serial.print("IP Local: ");
  Serial.println(net.localIP());
  Serial.printf("Servidor: %s:%d\n", SERVER_IP, SERVER_PORT);
  Serial.println("\n=== Aguardando conexão do PC... ===\n");
}

void loop() {
  // ECO: comando → medição (bate-volta)
  for (int i = 0; i < 3; i++) {
    currentMeasurement.q[i] = currentCommand.q_cmd[i];
  }
  currentMeasurement.gripper = currentCommand.gripper;
  
  // Atualiza biblioteca
  net.setMeasurement(currentMeasurement);
  
  // Mantém comunicação
  net.loop();
  
  delay(1);
}
