#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 4
#define TURBIDITY_PIN 34

// MOTOR DRIVER 1
#define M2_RPWM 26
#define M2_LPWM 25
#define M2_REN 27
#define M2_LEN 14

// MOTOR DRIVER 2
#define M1_RPWM 33
#define M1_LPWM 32
#define M1_REN 18
#define M1_LEN 19

// STEPPER (ULN2003)
#define IN1 5
#define IN2 17
#define IN3 16
#define IN4 13

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// PWM settings
const int freq = 20000;
const int resolution = 8;

const int chM1 = 0;
const int chM2 = 1;

int motor1Speed = 0;
int motor2Speed = 0;

String inputString = "";

// Stepper sequence
const int seq[8][4] = {
 {1,0,0,0},
 {1,1,0,0},
 {0,1,0,0},
 {0,1,1,0},
 {0,0,1,0},
 {0,0,1,1},
 {0,0,0,1},
 {1,0,0,1}
};

int stepIdx = 0;

#define TURN_STEPS 1024

void stepMotor(int dir)
{
  for(int i=0;i<TURN_STEPS;i++)
  {
    stepIdx += dir;

    if(stepIdx > 7) stepIdx = 0;
    if(stepIdx < 0) stepIdx = 7;

    digitalWrite(IN1,seq[stepIdx][0]);
    digitalWrite(IN2,seq[stepIdx][1]);
    digitalWrite(IN3,seq[stepIdx][2]);
    digitalWrite(IN4,seq[stepIdx][3]);

    delay(3);
  }

  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}

void setup()
{
  Serial.begin(115200);

  sensors.begin();

  pinMode(TURBIDITY_PIN,INPUT);

  // enable motors
  pinMode(M1_REN,OUTPUT);
  pinMode(M1_LEN,OUTPUT);
  pinMode(M2_REN,OUTPUT);
  pinMode(M2_LEN,OUTPUT);

  digitalWrite(M1_REN,HIGH);
  digitalWrite(M1_LEN,HIGH);
  digitalWrite(M2_REN,HIGH);
  digitalWrite(M2_LEN,HIGH);

  // pwm setup
  ledcSetup(chM1,freq,resolution);
  ledcAttachPin(M1_RPWM,chM1);

  ledcSetup(chM2,freq,resolution);
  ledcAttachPin(M2_RPWM,chM2);

  pinMode(M1_LPWM,OUTPUT);
  pinMode(M2_LPWM,OUTPUT);

  digitalWrite(M1_LPWM,LOW);
  digitalWrite(M2_LPWM,LOW);

  // stepper pins
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  Serial.println("System Ready");
  Serial.println("Commands:");
  Serial.println("M1 <0-255> -> Motor1 speed");
  Serial.println("M2 <0-255> -> Motor2 speed");
  Serial.println("R -> Turn Right");
  Serial.println("L -> Turn Left");
}

void loop()
{

  // SERIAL INPUT
  while(Serial.available())
  {
    char c = Serial.read();

    if(c == '\n')
    {

      if(inputString.startsWith("M1"))
      {
        int val = inputString.substring(3).toInt();

        if(val >=0 && val <=255)
        {
          motor1Speed = val;
          ledcWrite(chM1,motor1Speed);

          Serial.print("Motor1 Speed: ");
          Serial.println(motor1Speed);
        }
      }

      else if(inputString.startsWith("M2"))
      {
        int val = inputString.substring(3).toInt();

        if(val >=0 && val <=255)
        {
          motor2Speed = 255 - val;

          ledcWrite(chM2,motor2Speed);

          Serial.print("Motor2 Speed: ");
          Serial.println(val);
        }
      }

      else if(inputString == "R")
      {
        Serial.println("Turning Right");
        stepMotor(1);
      }

      else if(inputString == "L")
      {
        Serial.println("Turning Left");
        stepMotor(-1);
      }

      inputString = "";
    }
    else
    {
      inputString += c;
    }
  }

  // temperature
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);

  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" C");

  // turbidity
  int turbidityValue = analogRead(TURBIDITY_PIN);
  float voltage = turbidityValue * (3.3 / 4095.0);

  Serial.print("Turbidity Raw: ");
  Serial.println(turbidityValue);

  Serial.print("Voltage: ");
  Serial.println(voltage);

  if(voltage > 2.5)
    Serial.println("Water: Clear");
  else if(voltage > 1.5)
    Serial.println("Water: Slightly Cloudy");
  else
    Serial.println("Water: Dirty");

  Serial.println("-------------------------");

  delay(2000);
}