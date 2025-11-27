/*
 * ============================================================================
 * TESTE 3: CONTROLE BAIXO NÍVEL (ESP32 faz PID)
 * ============================================================================
 * 
 * O que faz:
 * - Recebe trajetória desejada (q_d, qd_d, qdd_d) e ganhos (Kp, Kd, Ki)
 * - Implementa controlador PID na ESP32
 * - Envia apenas posição medida de volta ao PC
 * 
 * Vantagens:
 * - Menor latência (controle local)
 * - Menos dados trafegados na rede
 * - Robustez contra perda de conexão
 * 
 * Desvantagens:
 * - Menos flexibilidade (ganhos devem ser ajustados)
 * - Processamento limitado da ESP32
 * 
 * Use para:
 * - Sistema final de produção
 * - Aplicações que exigem baixa latência
 * - Controle robusto mesmo com WiFi instável
 * 
 * ============================================================================
 */

#include "Esp32Link_PID.h"

// ===== CONFIGURAÇÃO WiFi =====
const char* SSID = "SUA_REDE";
const char* PASS = "SUA_SENHA";
const char* SERVER_IP = "192.168.1.100";
const uint16_t SERVER_PORT = 9000;
// =============================

// ===== CONFIGURAÇÃO PID =====
const float DT = 0.01;  // 10ms = 100Hz
// ============================

Esp32Link_PID net;

RobotCommand    currentCommand;
RobotMeasurement currentMeasurement;

// Estado do robô
float q_real[3] = {0.0, 1.396, 0.03};
float q_error_integral[3] = {0.0, 0.0, 0.0};
float q_error_last[3] = {0.0, 0.0, 0.0};

// Ganhos PID (serão atualizados pelo PC via set_gains)
float Kp = 0.05;
float Kd = 0.15;
float Ki = 0.001;

unsigned long lastControl = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== CONTROLE BAIXO NÍVEL (PID na ESP32) ===");
  
  // Inicializa
  for (int i = 0; i < 3; i++) {
    currentCommand.q_d[i] = q_real[i];
    currentMeasurement.q[i] = q_real[i];
  }
  currentCommand.gripper = 0;
  currentMeasurement.gripper = 0;
  
  // Callbacks
  net.onConnected([]() {
    Serial.println("[✓] PC conectado!");
  });
  
  net.onDisconnected([]() {
    Serial.println("[✗] PC desconectado");
  });
  
  net.onSetReference([](const RobotCommand& cmd) {
    // Recebe comando
    currentCommand = cmd;
    
    // Atualiza ganhos se foram enviados
    if (cmd.kp > 0) {
      Kp = cmd.kp;
      Kd = cmd.kd;
      Ki = cmd.ki;
      Serial.printf("[GANHOS] Kp=%.4f, Kd=%.4f, Ki=%.4f\n", Kp, Kd, Ki);
    }
    
    // Debug
    static int count = 0;
    if (++count >= 100) {
      Serial.printf("[REF] q_d=[%.3f, %.3f, %.3f]\n",
        cmd.q_d[0], cmd.q_d[1], cmd.q_d[2]);
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
  
  lastControl = millis();
}

void loop() {
  // Loop de controle PID
  unsigned long now = millis();
  if (now - lastControl >= (DT * 1000)) {
    lastControl = now;
    
    // ===== CONTROLADOR PID =====
    for (int i = 0; i < 3; i++) {
      // Erro
      float error = currentCommand.q_d[i] - q_real[i];
      
      // Termo P
      float P = Kp * error;
      
      // Termo I (com anti-windup)
      q_error_integral[i] += error * DT;
      q_error_integral[i] = constrain(q_error_integral[i], -10.0, 10.0);
      float I = Ki * q_error_integral[i];
      
      // Termo D
      float D = Kd * (error - q_error_last[i]) / DT;
      
      // Controle total
      float u = P + I + D;
      
      // Simula planta (modelo simplificado)
      // Em hardware real: u seria convertido em PWM/tensão
      q_real[i] += u * DT * 0.1;  // Ganho da planta
      
      // Limites físicos (adicione os seus)
      // q_real[0] = constrain(q_real[0], -3.14, 3.14);
      // q_real[1] = constrain(q_real[1], ...);
      // q_real[2] = constrain(q_real[2], ...);
      
      q_error_last[i] = error;
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

/*
 * COMO USAR ESTE CÓDIGO:
 * 
 * 1. Na interface Python, selecione "PID (Baixo Nível)"
 * 2. Configure os ganhos Kp, Kd, Ki
 * 3. Clique em "Conectar ESP32"
 * 4. Carregue este código na ESP32
 * 5. Execute a simulação
 * 
 * O que acontece:
 * - PC envia: trajetória desejada + ganhos
 * - ESP32 faz: controle PID local
 * - PC recebe: apenas posição medida
 * - PC plota: gráficos de erro e posição
 * 
 * Ajuste fino:
 * - Aumente Kp para resposta mais rápida (cuidado com overshoot)
 * - Aumente Kd para amortecer oscilações
 * - Ki elimina erro em regime permanente (use com cuidado)
 */
