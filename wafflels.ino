#include <LiquidCrystal.h>
#include <math.h>
#include <PID_v1.h>

float const A = 3.9083 * pow(10.0, -3.0);
float const B = -5.775 * pow(10.0, -7.0);
float const RES = 110.55;
float const RES_0 = 92.5;

float temp;

int const termPin = 0;
int const relayPin = 13;
int const potPin = 1;

double Setpoint, Input, Output;
double Kp = 2; //porportionell förändring
double Ki = 5; //Integration
double Kd = 1; //Derivering

PID tempPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(11, 12, 5, 4, 3, 2);

void setup() {
  windowStartTime = millis();

  Setpoint = 100;

  tempPID.SetOutputLimits(0, WindowSize);
  tempPID.SetMode(AUTOMATIC);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);
  lcd.begin(16, 2);
}

void loop() {
  lcd.clear();
  int readVal = analogRead(termPin);
  float voltage = readVal * (5.0 / 1023.0);
  float restistance = ((float) RES*(5.0-voltage))/voltage;
  float temp_pos = (-RES_0 * A + sqrt(pow(RES_0 * A, 2.0) - 4*RES_0 * B * (RES_0 - restistance)))/(2*RES_0*B);
  float temp_neg = (-RES_0 * A - sqrt(pow(RES_0 * A, 2.0) - 4*RES_0 * B * (RES_0 - restistance)))/(2*RES_0*B);
  lcd.print(temp_pos);
  lcd.setCursor(0,1);
  lcd.print(restistance);
  lcd.setCursor(9,0);
  lcd.print(voltage);
  lcd.setCursor(9,1);
  

  Input = temp_pos;
  tempPID.Compute();

  unsigned long now = millis();
  if (now-windowStartTime > WindowSize) {
    windowStartTime += WindowSize;
  } if(Output > now - windowStartTime) digitalWrite(relayPin, HIGH);
  else digitalWrite(relayPin, LOW);
  

  int potVal = analogRead(potPin);
  int targTemp = map(potVal, 0, 1023, 100, 251);
  lcd.print(targTemp);
  delay(15);
}

