#include <Wire.h>
#include <ESP32Servo.h>
#include <Esp32Link_PID.h>

Esp32Link_PID net;

// ============================= Configuração WiFi =============================
const char* STA_SSID = "SUA_REDE";
const char* STA_PASS = "SUA_SENHA";
const char* SERVER_IP = "IP_DO_PC";
const uint16_t SERVER_PORT = 9000;
// =============================================================================

const int PINO_SERVO = 27;
const int ANGULO_FECHAR_GARRA = 20;
const int ANGULO_ABRIR_GARRA  = 110;
const int ZERO_OFFSET = 1740; 
const float DEADBAND_DEG = 2.0; 

#define SDA_PIN 21       
#define SCL_PIN 22       
#define SERVO_PIN 25  

#define PINO_POT_1 32 // Potenciômetro do Motor 1
#define PINO_POT_2 33 // Potenciômetro do Motor 2

#define MIN_LEITURA_ADC 142  
#define MAX_LEITURA_ADC 4012 
#define POS_MIN_GRAUS 0.0
#define POS_MAX_GRAUS 180.0

#define ENABLE_A 19 
#define IN1 18      
#define IN2 5       

#define ENABLE_B 17 
#define IN3 16      
#define IN4 4       

#define MAX_PWM 200

#define AS5600_ADDR 0x36

double SETPOINT_M1 = 80.0;
double SETPOINT_M2 = 110.0;

double KP_E = 0.01;
double KI_E = 0.02;  
double KD_E = 2.5;

float KP_M = 6.0;
float KI_M = 3.0; 
float KD_M = 0.0;

#define DEAD_ZONE_PWM 40
#define TOLERANCIA_ERRO 0.5

double INTEGRAL_M1 = 0.0;
double previousError_M1 = 0.0;
double INTEGRAL_M2 = 0.0;      
double previousError_M2 = 0.0; 
unsigned long lastLoopTime = 0;

Servo myServo;
double setpoint = 0.0; 
double input, output;
double error, lastError = 0;
double integral = 0;
unsigned long lastTime = 0;
double servoPos = 90.0; // Posição lógica inicial do servo

float readAS5600Deg();

#define INTEGRAL_LIMIT 100.0 

Servo garraServo;

double lerPosicaoAtualEmGraus(int pino_pot) { 
  int leituraADC = analogRead(pino_pot);

  double posicaoGraus = (leituraADC - MIN_LEITURA_ADC) * (POS_MAX_GRAUS - POS_MIN_GRAUS) / (MAX_LEITURA_ADC - MIN_LEITURA_ADC) + POS_MIN_GRAUS;
  posicaoGraus = constrain(posicaoGraus, POS_MIN_GRAUS, POS_MAX_GRAUS);

  return posicaoGraus;
}

void acionarMotor_1(int pwm) {
  if (pwm > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    int pwmFinal = map(pwm, 1, MAX_PWM, DEAD_ZONE_PWM, MAX_PWM);
    analogWrite(ENABLE_A, pwmFinal);
  } else if (pwm < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    int pwmFinal = map(abs(pwm), 1, MAX_PWM, DEAD_ZONE_PWM, MAX_PWM);
    analogWrite(ENABLE_A, pwmFinal);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENABLE_A, 0);
  }
}

void acionarMotor_2(int pwm2) {
  if (pwm2 > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    int pwmFinal2 = map(pwm2, 1, MAX_PWM, DEAD_ZONE_PWM, MAX_PWM);
    analogWrite(ENABLE_B, pwmFinal2);
  } else if (pwm2 < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    int pwmFinal2 = map(abs(pwm2), 1, MAX_PWM, DEAD_ZONE_PWM, MAX_PWM);
    analogWrite(ENABLE_B, pwmFinal2);
  } else {
    digitalWrite(IN3, LOW);  
    digitalWrite(IN4, LOW);  
    analogWrite(ENABLE_B, 0);
  }
}

void verificarComandosSerial() {
  if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    comando.toUpperCase();

    if (comando.startsWith("FECHAR GARRA")) {
      Serial.println(">> COMANDO: Fechando Garra...");
      garraServo.write(ANGULO_FECHAR_GARRA); 
      Serial.print(">> Servo movido para "); Serial.println(ANGULO_FECHAR_GARRA);
    } 
    else if (comando.startsWith("ABRIR GARRA")) {
      Serial.println(">> COMANDO: Abrindo Garra...");
      garraServo.write(ANGULO_ABRIR_GARRA); 
      Serial.print(">> Servo movido para "); Serial.println(ANGULO_ABRIR_GARRA);
    }
    else if (comando.startsWith("M1 ")) {
       SETPOINT_M1 = comando.substring(3).toDouble();
       Serial.print(">> Novo Setpoint M1: "); Serial.println(SETPOINT_M1);
    }
    else if (comando.startsWith("M2 ")) {
       SETPOINT_M2 = comando.substring(3).toDouble();
       Serial.print(">> Novo Setpoint M2: "); Serial.println(SETPOINT_M2);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  Wire.begin(SDA_PIN, SCL_PIN);

  myServo.setPeriodHertz(50); 
  myServo.attach(SERVO_PIN, 500, 2400); 

  Wire.beginTransmission(AS5600_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERRO: AS5600 não encontrado! Verifique cabos.");
    while (1);
  }

  Serial.println("--- SISTEMA INICIADO COM ZONA MORTA ---");
  Serial.print("Zona Morta configurada para: +/- ");
  Serial.print(DEADBAND_DEG);
  Serial.println(" graus.");
  Serial.println("Digite um angulo (ex: 90) no Serial para mover.");

  lastTime = millis();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENABLE_A, OUTPUT); 
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENABLE_B, OUTPUT); 
  
  Serial.println("Inicializando Hardware...");
  garraServo.attach(PINO_SERVO);
  garraServo.write(ANGULO_ABRIR_GARRA);

  lastLoopTime = millis();
  
  Serial.println("--- SISTEMA PRONTO ---");
  Serial.println("Comandos disponíveis:");
  Serial.println("1. ABRIR GARRA");
  Serial.println("2. FECHAR GARRA");
  Serial.println("3. M1 [angulo] (Ex: M1 90)");
  Serial.println("4. M2 [angulo] (Ex: M2 45)");
}

void loop() {
  verificarComandosSerial();

  if (Serial.available() > 0) {
    float newTarget = Serial.parseFloat();
    while(Serial.available()) Serial.read(); 
    setpoint = newTarget;
    Serial.print("Novo Alvo: ");
    Serial.println(setpoint);
  }

  input = readAS5600Deg();

  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);
  
  if (timeChange >= 20) { // Executa a aprox 50Hz
    
    // Calcula Erro
    error = setpoint - input;
    
    // Tratamento de menor caminho (Wrap-around -180 a 180)
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    // ==========================================================
    // LÓGICA DA ZONA MORTA (DEADBAND)
    // ==========================================================
    if (abs(error) < DEADBAND_DEG) {
       // Se o erro é pequeno, fingimos que é zero.
       // Isso impede o motor de ficar "sarrando" ou oscilando.
       output = 0;
       integral = 0; // Zera integral para não acumular força parada
       // Serial.println("No alvo (Zona Morta)"); // Descomente para debug
       
    } else {
       // Se o erro é grande, calcula o PID normal
       
       integral += (error * timeChange);
       // Trava de segurança da integral (Anti-Windup)
       if (integral > 50) integral = 50;
       if (integral < -50) integral = -50;

       double derivative = (error - lastError) / timeChange;

       output = (KP_E * error) + (KI_E * integral) + (KD_E * derivative);
    }
    // ==========================================================

    // Aplica a saída à posição do servo
    // Se output for 0 (zona morta), servoPos não muda e o motor fica quieto.
    servoPos = servoPos + output; 

    // Limites de segurança do hardware
    servoPos = constrain(servoPos, 0, 180); 

    // Envia ao Motor
    myServo.write(servoPos);

    lastError = error;
    lastTime = now;
  }

  double timeDelta = (double)(now - lastLoopTime) / 1000.0;
  
  if (timeDelta < 0.001) return; 

  double input_1 = lerPosicaoAtualEmGraus(PINO_POT_1);

  double erro_1 = SETPOINT_M1 - input_1;
  double output_pwm_1 = 0.0;

  if (abs(erro_1) < TOLERANCIA_ERRO) {
    output_pwm_1 = 0.0;
    INTEGRAL_M1 = 0.0;
  } else {
    double P_term_1 = KP_M * erro_1;
    INTEGRAL_M1 += erro_1 * timeDelta;
    INTEGRAL_M1 = constrain(INTEGRAL_M1, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    double I_term_1 = KI_M * INTEGRAL_M1;
    double derivative_1 = (erro_1 - previousError_M1) / timeDelta;
    double D_term_1 = KD_M * derivative_1;
    output_pwm_1 = P_term_1 + I_term_1 + D_term_1;
  }
  output_pwm_1 = constrain(output_pwm_1, -MAX_PWM, MAX_PWM);
  acionarMotor_1((int)round(output_pwm_1));
  previousError_M1 = erro_1; 

  double input_2 = lerPosicaoAtualEmGraus(PINO_POT_2);

  double erro_2 = SETPOINT_M2 - input_2;
  double output_pwm_2 = 0.0;

  if (abs(erro_2) < TOLERANCIA_ERRO) {
    output_pwm_2 = 0.0;
    INTEGRAL_M2 = 0.0;
  } else {
    double P_term_2 = KP_M * erro_2; 
    INTEGRAL_M2 += erro_2 * timeDelta;
    INTEGRAL_M2 = constrain(INTEGRAL_M2, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    double I_term_2 = KI_M * INTEGRAL_M2;
    double derivative_2 = (erro_2 - previousError_M2) / timeDelta;
    double D_term_2 = KD_M * derivative_2;
    output_pwm_2 = P_term_2 + I_term_2 + D_term_2;
  }
  output_pwm_2 = constrain(output_pwm_2, -MAX_PWM, MAX_PWM);
  acionarMotor_2((int)round(output_pwm_2));
  previousError_M2 = erro_2;

  lastLoopTime = now;

  // Serial.print("M1_Set:"); Serial.print(SETPOINT_M1);
  // Serial.print(",M1_In:"); Serial.print(input_1);
  // Serial.print(" | "); 
  // Serial.print("M2_Set:"); Serial.print(SETPOINT_M2);
  // Serial.print(",M2_In:"); Serial.println(input_2);
}

float readAS5600Deg() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 2);
  
  if (Wire.available() >= 2) {
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();
    uint16_t rawAngle = (highByte << 8) | lowByte; 

    // Calibração
    long correctedRaw = rawAngle - ZERO_OFFSET;

    // Correção circular (0-4095)
    if (correctedRaw < 0) correctedRaw += 4096;
    if (correctedRaw >= 4096) correctedRaw -= 4096;

    return (float)correctedRaw * 0.087890625; 
  }
  return 0.0;
}