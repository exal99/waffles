#include <LiquidCrystal.h>
#include <math.h>

float const A = 3.9083 * pow(10.0, -3.0);
float const B = -5.775 * pow(10.0, -7.0);
float const RES = 99;
float const RES_0 = 100;

int const termPin = 0;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(11, 12, 5, 4, 3, 2);

void setup() {
  lcd.begin(16, 2);
}

void loop() {
  lcd.clear();
  int readVal = analogRead(termPin);
  float voltage = readVal * (5.0 / 1023.0);
  float restistance = ((float) RES*(5.0-voltage))/voltage;
  float temp_pos = (-RES_0 * A + sqrt(pow(RES_0 * A, 2.0) - 4*RES_0 * B * (RES_0 - restistance)))/(2*RES_0*B);
  float temp_neg = (-RES_0 * A - sqrt(pow(RES_0 * A, 2.0) - 4*RES_0 * B * (RES_0 - restistance)))/(2*RES_0*B);
  lcd.print(voltage);
  lcd.setCursor(0,1);
  lcd.print(restistance);
  delay(500);
}

