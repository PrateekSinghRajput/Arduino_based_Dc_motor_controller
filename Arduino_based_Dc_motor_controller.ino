#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define potentiometer  A0  //10k Variable Resistor
#define red 4
#define yellow 7
#define bt_F A1 // Clockwise Button
#define bt_S A2 // Stop Button
#define bt_B A3 // Anticlockwise Button

#define M1_Ena 11 // Enable1 L298 for PWM
#define M1_in1 10 // In1  L298 for Clockwise
#define M1_in2 9 // In2  L298 for Anticlockwise

#define M1_Ena2 3 // Enable1 L298 for PWM
#define M1_in3 5 // In1  L298 for Clockwise
#define M1_in4 6 // In2  L298 for Anticlockwise

int read_ADC = 0;
int duty_cycle;
int duty_cycle_lcd;
int set = 0;

void setup() {
  Serial.begin(9600);// initialize serial communication at 9600 bits per second:
  lcd.init();
  lcd.backlight();
  pinMode(potentiometer, INPUT);
  pinMode(red, OUTPUT);
  pinMode(yellow, OUTPUT);
  digitalWrite(red, LOW);
  digitalWrite(yellow, LOW);

  pinMode(bt_F, INPUT_PULLUP);
  pinMode(bt_S, INPUT_PULLUP);
  pinMode(bt_B, INPUT_PULLUP);

  pinMode(M1_Ena, OUTPUT);
  pinMode(M1_in1, OUTPUT);
  pinMode(M1_in2, OUTPUT);

  pinMode(M1_Ena2, OUTPUT);
  pinMode(M1_in3, OUTPUT);
  pinMode(M1_in4, OUTPUT);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("   WELCOME To ");
  lcd.setCursor(0, 1);
  lcd.print("JustDoElectronic");
  delay(2000); // Waiting for a while
  lcd.clear();
}

void loop() {
  read_ADC = analogRead(potentiometer);
  duty_cycle = map(read_ADC, 0, 1023, 0, 255);
  duty_cycle_lcd = map(read_ADC, 0, 1023, 0, 100);

  analogWrite(M1_Ena, duty_cycle);
  analogWrite(M1_Ena2, duty_cycle);

  lcd.setCursor(0, 0);
  lcd.print("Duty Cycle: ");
  lcd.print(duty_cycle_lcd);
  lcd.print("%  ");

  if (digitalRead (bt_F) == 0) {
    set = 1;
  }
  if (digitalRead (bt_S) == 0) {
    set = 0;
  }
  if (digitalRead (bt_B) == 0) {
    set = 2;
  }


  lcd.setCursor(0, 1);

  if (set == 0) {
    lcd.print("      Stop      ");
    digitalWrite(M1_in1, LOW);
    digitalWrite(M1_in2, LOW);
    digitalWrite(M1_in3, LOW);
    digitalWrite(M1_in4, LOW);
    digitalWrite(red, LOW);
    digitalWrite(yellow, LOW);
  }

  if (set == 1) {
    lcd.print("    Clockwise   ");
    digitalWrite(M1_in1, HIGH);
    digitalWrite(M1_in2, LOW);
    digitalWrite(M1_in3, HIGH);
    digitalWrite(M1_in4, LOW);
    digitalWrite(red, HIGH);
  digitalWrite(yellow, LOW);
  }

  if (set == 2) {
    lcd.print(" Anticlockwise  ");
    digitalWrite(M1_in1, LOW);
    digitalWrite(M1_in2, HIGH);
    digitalWrite(M1_in3, LOW);
    digitalWrite(M1_in4, HIGH);
    digitalWrite(red, LOW);
  digitalWrite(yellow, HIGH);
  }

  delay(50);
}
