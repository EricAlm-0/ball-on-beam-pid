#include <Arduino.h>
#include <Servo.h>

// ── Pinnar ──────────────────────────────────────────
#define TRIG_PIN  6
#define ECHO_PIN  7
#define SERVO_PIN 9

// ── Inställningar ────────────────────────────────────
float setpoint     = 11.0;
float rannans_langd = 18.0;
int   servoMid     = 51;
int   maxTipp      = 25;

// ── PID-parametrar ───────────────────────────────────
float Kp = 2.3;
float Ki = 0.0;
float Kd = 0.6;

// ── EMA-filter ──────────────────────────────────────
// Alpha styr avvägningen mellan brusreducering och responstid.
// 0.1 = mycket utjämning, 0.4 = snabbare respons.
// Börja med 0.2 och justera experimentellt.
float alpha        = 0.2;
float filtreradAvstand = -1.0;  // Initieras vid första giltig mätning

// ── Interna variabler ────────────────────────────────
Servo myServo;
float prevError  = 0;
float integral   = 0;
unsigned long prevTime = 0;

// ── Funktioner ───────────────────────────────────────

float matAvstand() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long varaktighet = pulseIn(ECHO_PIN, HIGH, 30000);
  if (varaktighet == 0) return -1;

  float avstand = varaktighet * 0.0343 / 2.0;

  if (avstand < 2 || avstand > rannans_langd + 5) return -1;
  return avstand;
}

// Applicerar EMA-filter på en ny mätning.
// Hanterar initiering automatiskt vid första giltig mätning.
float emaFilter(float nyMatning) {
  if (filtreradAvstand < 0) {
    // Initiering: sätt filtret till första giltiga mätningen
    // istället för att börja på noll (undviker en kraftig transient).
    filtreradAvstand = nyMatning;
    return filtreradAvstand;
  }
  filtreradAvstand = alpha * nyMatning + (1.0 - alpha) * filtreradAvstand;
  return filtreradAvstand;
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  myServo.attach(SERVO_PIN);
  myServo.write(servoMid);

  prevTime = millis();
  delay(500);

  Serial.println("=== Boll-balansering startar ===");
  Serial.print("Målposition: ");
  Serial.print(setpoint);
  Serial.println(" cm");
  Serial.print("EMA alpha: ");
  Serial.println(alpha);
}

void loop() {
  float avstand = matAvstand();

  if (avstand < 0) {
    Serial.println("Ingen boll detekterad");
    delay(20);
    return;
  }

  // Applicera EMA-filtret på råmätningen
  float avstandFiltrerad = emaFilter(avstand);

  // ── Tidsdelta ──────────────────────────────────────
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;
  if (dt < 0.001) dt = 0.02;

  // ── PID-beräkning (använder filtrerat värde) ───────
  float error = setpoint - avstandFiltrerad;

  integral += error * dt;
  integral = constrain(integral, -20, 20);

  float derivative = (error - prevError) / dt;
  prevError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;

  // ── Servo-styrning ─────────────────────────────────
  output = constrain(output, -maxTipp, maxTipp);
  myServo.write(servoMid - (int)output);

  // ── Serial Plotter ─────────────────────────────────
  // Råvärdet visas bredvid det filtrerade för jämförelse.
  Serial.print("Setpoint:");        Serial.print(setpoint);
  Serial.print(" Rå:");             Serial.print(avstand);
  Serial.print(" Filtrerad:");      Serial.print(avstandFiltrerad);
  Serial.print(" Error:");          Serial.print(error);
  Serial.print(" Output:");         Serial.println(output);

  delay(20); // 50 Hz
}