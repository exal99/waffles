#include <LiquidCrystal.h>
#include <math.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

float const A = 3.9083 * pow(10.0, -3.0);
float const B = -5.775 * pow(10.0, -7.0);
float const RES = 110.25;
//float const RES_0 = 92.5;
float const RES_0 = 100.0;

float temp;

int const termPin = 0;
int const relayPin = 9;
int const potPin = 1;

double Setpoint, Input, Output;
double Kp = 3.4; //porportionell förändring
double Ki = 0.08; //Integration
double Kd = 0; //Derivering

// 0.66, 0.1 & 1.27, 0.03 & 1.13, 0.01
PID tempPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID_ATune aTunePID(&Input, &Output);

int WindowSize = 10000;
unsigned long windowStartTime;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(11, 12, 5, 4, 3, 2);

void setup() {
  windowStartTime = millis();

  Setpoint = 150;

  tempPID.SetOutputLimits(0, WindowSize);
  tempPID.SetMode(AUTOMATIC);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  lcd.begin(16, 2);
  Serial.begin(9600);
}

void loop() {
  lcd.clear();
  int readVal = analogRead(termPin);
  float voltage = readVal * (5 / 1023.0);
  float restistance = ((float) RES*(3.3-voltage))/voltage;
  float temp_pos = (-RES_0 * A + sqrt(pow(RES_0 * A, 2.0) - 4*RES_0 * B * (RES_0 - restistance)))/(2*RES_0*B);
  float temp_neg = (-RES_0 * A - sqrt(pow(RES_0 * A, 2.0) - 4*RES_0 * B * (RES_0 - restistance)))/(2*RES_0*B);
  lcd.print(round(temp_pos));
  lcd.setCursor(0,1);
  lcd.print(restistance);
  lcd.setCursor(9,0);
  lcd.print(voltage);
  lcd.setCursor(9,1);
  

  Input = round(temp_pos);
  tempPID.Compute();

  unsigned long now = millis();
  if (now-windowStartTime > WindowSize) {
    windowStartTime += WindowSize;
  } if(Output > now - windowStartTime) digitalWrite(relayPin, HIGH);
  else digitalWrite(relayPin, LOW);
  

  int potVal = analogRead(potPin);
  int targTemp = map(potVal, 0, 1023, 100, 251);
  //Setpoint = targTemp;
  lcd.print(targTemp);
  int res = aTunePID.Runtime();
  if (res == 1) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("A_TUNE COMPLEAT!");
    Serial.print("Kp: ");
    Serial.println(aTunePID.GetKp());
    Serial.print("Ki: ");
    Serial.println(aTunePID.GetKi());
    Serial.print("Kd: ");
    Serial.println(aTunePID.GetKd());
    lcd.setCursor(0,1);
    lcd.print(aTunePID.GetKp());
    lcd.setCursor(4, 1);
    lcd.print(aTunePID.GetKi());
    lcd.setCursor(9,1);
    lcd.print(aTunePID.GetKd());
  }
  
  delay(100);
}

