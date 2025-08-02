#include <Servo.h>
#include <Wire.h>
#include <MS5xxx.h>

// Fan control pins
const int fan_enable_pin = 29; // MOSFET gate pin
const int fan_pwm_pin    = 28; // PWM signal to fan (blue wire)

// Altimeter
MS5xxx sensor(&Wire);

// Servo control
Servo servo1, servo2, servo3;
const int servoPins[3] = {5, 6, 7};

// States
bool servo1Activated = false;
bool servo2Activated = false;
bool servo3Activated = false;
bool fanOn = false;
bool shutdownTriggered = false;
unsigned long servo3OpenedTime = 0;

// Altitude thresholds
const double level1 = 300.0;   // Servo 1
const double level2 = 346.0;   // Servo 2
const double level3 = 448.0;   // Servo 3 and Fan
const unsigned long shutdownDelay = 10000; // 10 seconds

void setup() {
  Serial.begin(9600);
  // Removed "while (!Serial)" for portable deployment

  // Altimeter setup
  if (sensor.connect() > 0) {
    Serial.println("Error connecting to altimeter...");
    while (1);
  }

  // Fan setup
  pinMode(fan_enable_pin, OUTPUT);
  pinMode(fan_pwm_pin, OUTPUT);
  digitalWrite(fan_enable_pin, LOW);
  analogWrite(fan_pwm_pin, 0);

  // Servo setup
  servo1.attach(servoPins[0]);
  servo2.attach(servoPins[1]);
  servo3.attach(servoPins[2]);
  servo1.write(0);
  servo2.write(0);
  servo3.write(0);

  Serial.println("System ready. Reading altitude...");
}

void loop() {
  sensor.Readout();
  double pres = sensor.GetPres();
  double alt = altitude(pres);


  Serial.print("Raw Pressure [Pa]: ");
  Serial.println(pres);
  Serial.print("Altitude [m]: ");
  Serial.println(alt);

  // Servo 1 logic
  if (!servo1Activated && alt >= (level1 - 1) && alt <= level2 ) {
    servo1.write(115);
    servo1Activated = true;
    Serial.println("Servo 1 opened at 163m.");
  }

  // Servo 2 logic
  if (!servo2Activated && alt >= (level2 - 1) && alt <= level3) {
    servo2.write(115);
    servo2Activated = true;
    Serial.println("Servo 2 opened at 164m.");

    if (servo1Activated) {
      servo1.write(0);
      Serial.println("Servo 1 closed.");
    }
  }

  // Servo 3 and fan logic
  if (!servo3Activated && alt >= (level3 - 1) ) {
    servo3.write(115);
    servo3Activated = true;
    servo3OpenedTime = millis();
    Serial.println("Servo 3 opened at 165m.");

    if (servo2Activated) {
      servo2.write(0);
      Serial.println("Servo 2 closed.");
    }

    if (!fanOn) {
      digitalWrite(fan_enable_pin, HIGH);
      analogWrite(fan_pwm_pin, 128); // 50% PWM
      fanOn = true;
      Serial.println("Fan turned on at 50%.");
    }
  }

  // Shutdown 10 seconds after servo 3 opens
  if (servo3Activated && !shutdownTriggered && millis() - servo3OpenedTime >= shutdownDelay) {
    servo3.write(0);
    analogWrite(fan_pwm_pin, 0);
    digitalWrite(fan_enable_pin, LOW);
    shutdownTriggered = true;
    fanOn = false;
    Serial.println("Shutdown: Servo 3 closed and fan turned off after 10 seconds.");
  }

  delay(250);
}

double altitude(double P) {
  return 44330.0 * (1 - pow(P / 101325.0, 1.0 / 5.255));
}
