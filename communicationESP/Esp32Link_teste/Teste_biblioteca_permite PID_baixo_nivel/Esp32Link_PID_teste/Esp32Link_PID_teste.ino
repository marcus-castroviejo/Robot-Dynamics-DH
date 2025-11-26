#include <Esp32Link_PID.h> 

// ======= Configuração da rede =======
const char* STA_SSID  = "iPhone de Marcus Vinicius";      // nome da rede
const char* STA_PASS  = "zv8Z-bSeC-R8g4-eUi7";        // senha
const char* SERVER_IP = "172.20.10.3";   // IP do PC
const uint16_t SERVER_PORT = 9000;         // porta
// ==============================

Esp32Link_PID net;

// Variável para guardar o COMANDO (trajetória) que o PC manda
RobotCommand currentCommand; 
    
// Variável para guardar a MEDIÇÃO (posição real) que vamos enviar
RobotMeasurement currentMeasurement;


void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[ESP] Inicializando (Projeto 3)...");

  // ===== ADICIONE AQUI: Inicializar com posição padrão =====
  currentCommand.q_d[0] = 0.0;        // q1 = 0 rad
  currentCommand.q_d[1] = 1.396;      // q2 = 80° em radianos
  currentCommand.q_d[2] = 0.03;       // d3 = 3 cm em metros
  currentCommand.gripper = 0;
  
  // Inicializar medição também
  for (int i = 0; i < 3; i++) {
    currentMeasurement.q[i] = currentCommand.q_d[i];
  }
  currentMeasurement.gripper = currentCommand.gripper;
  // ========================================================
  // --- 1. Configurar os Callbacks  ---
  net.onConnected([](){
    Serial.println("[ESP] Conectado ao servidor do PC!");
  });

  net.onDisconnected([](){
    Serial.println("[ESP] Desconectado do servidor.");
  });

  //Recebe o pacote de COMANDO (trajetória completa)
  net.onSetReference([](const RobotCommand& newCommand){
    
    // Copia o comando que chegou para a nossa variável global
    currentCommand = newCommand; 

    // --- Imprime os dados da trajetória para DEBUG ---
    Serial.printf("[ESP] CMD (q_d): %.3f, %.3f, %.3f\n", 
      currentCommand.q_d[0], 
      currentCommand.q_d[1], 
      currentCommand.q_d[2]
    );
    Serial.printf("[ESP] CMD (qd_d): %.3f, %.3f, %.3f\n", 
      currentCommand.qd_d[0], 
      currentCommand.qd_d[1], 
      currentCommand.qd_d[2]
    );
    Serial.printf("[ESP] CMD (qdd_d): %.3f, %.3f, %.3f\n", 
      currentCommand.qdd_d[0], 
      currentCommand.qdd_d[1], 
      currentCommand.qdd_d[2]
    );
    Serial.printf("[ESP] CMD (gripper): %d\n", 
      currentCommand.gripper
    );

    Serial.printf("[ESP] CMD PID: P=%.4f  D=%.4f  I=%.4f\n", 
      currentCommand.kp, 
      currentCommand.kd, 
      currentCommand.ki
    );
  });

  // --- 2. Conectar no WiFi e no Servidor ---
  Serial.println("[ESP] Conectando ao WiFi...");
  if (!net.beginClient(STA_SSID, STA_PASS, SERVER_IP, SERVER_PORT)) {
    Serial.println("[ESP] Falha ao conectar no Wi-Fi. Travado.");
    while (true) { delay(1000); }
  }

  Serial.print("[ESP] Conectado! IP local: "); Serial.println(net.localIP());
  Serial.printf("[ESP] Servidor do PC: %s:%u\n", SERVER_IP, SERVER_PORT);
}

void loop() {
  
  // --- Lógica de "Bate-Volta" (Eco) ---
  // (Simula que o robô obedeceu perfeitamente aos comandos de posição)

  // 1. Simula a Posição do Robô
  for (int i = 0; i < 3; i++) {
    currentMeasurement.q[i] = currentCommand.q_d[i];
  }

  // 2. Simula a Posição da Garra
  currentMeasurement.gripper = currentCommand.gripper;

  // 3. Entrega a MEDIÇÃO (simulada) para a biblioteca
  net.setMeasurement(currentMeasurement);

  // 4. Mantém a rede funcionando
  net.loop();

  // 5. Pausa
  delay(1); 
}