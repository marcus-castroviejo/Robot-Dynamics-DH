#include <Esp32Link.h> // Inclui a biblioteca que acabamos de atualizar

// ======= Configuração da rede =======
const char* STA_SSID  = "Aron's web";      
const char* STA_PASS  = "25122512";        
const char* SERVER_IP = "10.224.214.176"; 
const uint16_t SERVER_PORT = 9000;         
// ==============================

// Cria o objeto da biblioteca de rede
Esp32Link net;

// q_cmd, gripper o PC manda
RobotCommand currentCommand; 
    
// q_real, gripper_real enviar para o PC
RobotMeasurement currentMeasurement;


void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[ESP] Inicializando...");

  // --- 1. Configurar os Callbacks  ---
  net.onConnected([](){
    Serial.println("[ESP] Conectado ao servidor do PC!");
  });

  net.onDisconnected([](){
    Serial.println("[ESP] Desconectado do servidor.");
  });

  //Recebe o pacote (q_cmd, gripper)
  net.onSetReference([](const RobotCommand& newCommand){
    
    // Copia o comando
    currentCommand = newCommand; 

    // Mostra no Serial Monitor o valor de q_cmd
    Serial.printf("[ESP] CMD do PC (q_cmd): %.3f, %.3f, %.3f\n", 
      currentCommand.q_cmd[0], 
      currentCommand.q_cmd[1], 
      currentCommand.q_cmd[2]
    );

    // Mostra o comando da garra
    Serial.printf("[ESP] CMD do PC (gripper): %d\n", 
      currentCommand.gripper
    );
  });

  // --- Conectar no WiFi e no Servidor ---
  Serial.println("[ESP] Conectando ao WiFi...");
  if (!net.beginClient(STA_SSID, STA_PASS, SERVER_IP, SERVER_PORT)) {
    Serial.println("[ESP] Falha ao conectar no Wi-Fi. Travado.");
    while (true) { delay(1000); } // Trava aqui se não conseguir
  }

  Serial.print("[ESP] Conectado! IP local: "); Serial.println(net.localIP());
  Serial.printf("[ESP] Servidor do PC: %s:%u\n", SERVER_IP, SERVER_PORT);
}

void loop() {
  
  // Copia o 'q_cmd' (comando) para o 'q' (medição)
  for (int i = 0; i < 3; i++) {
    currentMeasurement.q[i] = currentCommand.q_cmd[i];
  }

  // Copia o 'gripper' (comando) para o 'gripper' (medição)
  currentMeasurement.gripper = currentCommand.gripper;

  // A biblioteca vai enviar isso quando o PC pedir 'get_meas'
  net.setMeasurement(currentMeasurement);

  // 4. Mantém a rede funcionando (recebe comandos, envia medições)
  net.loop();

  // 5. Pausa
  delay(1); 
}