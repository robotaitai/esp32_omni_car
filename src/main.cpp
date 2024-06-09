#include <Arduino.h>

// Pin definitions
#define LF_L298N_ENA 25
#define LF_L298N_IN1 26
#define LF_L298N_IN2 27

#define RR_L298N_ENA 32
#define RR_L298N_IN1 33
#define RR_L298N_IN2 13

#define LR_L298N_ENA 12
#define LR_L298N_IN1 2
#define LR_L298N_IN2 4

#define RF_L298N_ENA 23
#define RF_L298N_IN1 25
#define RF_L298N_IN2 21

// Define PWM channels
#define LF_PWM_CHANNEL 0
#define RR_PWM_CHANNEL 1
#define LR_PWM_CHANNEL 2
#define RF_PWM_CHANNEL 3

// Define PWM frequency and resolution
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);

  // Set motor control pins as outputs
  pinMode(LF_L298N_IN1, OUTPUT);
  pinMode(LF_L298N_IN2, OUTPUT);

  pinMode(RR_L298N_IN1, OUTPUT);
  pinMode(RR_L298N_IN2, OUTPUT);

  pinMode(LR_L298N_IN1, OUTPUT);
  pinMode(LR_L298N_IN2, OUTPUT);

  pinMode(RF_L298N_IN1, OUTPUT);
  pinMode(RF_L298N_IN2, OUTPUT);

  // Configure PWM channels
  ledcSetup(LF_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(RR_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(LR_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(RF_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);

  // Attach PWM channels to pins
  ledcAttachPin(LF_L298N_ENA, LF_PWM_CHANNEL);
  ledcAttachPin(RR_L298N_ENA, RR_PWM_CHANNEL);
  ledcAttachPin(LR_L298N_ENA, LR_PWM_CHANNEL);
  ledcAttachPin(RF_L298N_ENA, RF_PWM_CHANNEL);
}

void turnMotor(int enaChannel, int in1, int in2, int direction)
{
  digitalWrite(in1, direction > 0 ? HIGH : LOW);
  digitalWrite(in2, direction > 0 ? LOW : HIGH);
  ledcWrite(enaChannel, 255); // Set PWM duty cycle to maximum (100%)
  delay(2000);                // Turn motor for 2 seconds
  ledcWrite(enaChannel, 0);   // Stop the motor
}

void loop()
{
  // Turn each wheel one at a time for 2 seconds
  Serial.println("Turning LF wheel");
  turnMotor(LF_PWM_CHANNEL, LF_L298N_IN1, LF_L298N_IN2, 1);

  Serial.println("Turning RR wheel");
  turnMotor(RR_PWM_CHANNEL, RR_L298N_IN1, RR_L298N_IN2, 1);

  Serial.println("Turning LR wheel");
  turnMotor(LR_PWM_CHANNEL, LR_L298N_IN1, LR_L298N_IN2, 1);

  Serial.println("Turning RF wheel");
  turnMotor(RF_PWM_CHANNEL, RF_L298N_IN1, RF_L298N_IN2, 1);

  delay(5000); // Wait 5 seconds before repeating
}
