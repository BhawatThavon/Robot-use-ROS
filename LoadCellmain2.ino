#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "HX711.h"

const int LOADCELL_DOUT_PIN = 2, LOADCELL_SCK_PIN = 3, LCD_ADDRESS = 0x27, LCD_COLUMNS = 20, LCD_ROWS = 4, BUZZER_PIN = 4, LED_PIN = 5;
const int over = 8;
const int under = 9;
const int error = 10;
const int over2 = 11;
const int under2 = 12;
const int error2 = 13;
float oldweight = 0.0;
float weight = 0.0;
int x = 0;
//unsigned long prevMillis2 = 0;
//unsigned long curMillis2 = 0;

HX711 scale;
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);
unsigned long prevMillis = 0;
unsigned long curMillis = 0;
const long interval = 200;
bool ledState = false;

void setup() {
  Serial.begin(57600);
  digitalWrite(under, HIGH);
  digitalWrite(under2, HIGH);
  lcd.init();
  lcd.backlight();
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(over, OUTPUT);
  pinMode(under, OUTPUT);
  pinMode(error, OUTPUT);
  pinMode(over2, OUTPUT);
  pinMode(under2, OUTPUT);
  pinMode(error2, OUTPUT);
  lcd.print("Initializing");
  lcd.setCursor(0, 1);
  lcd.print("the scale");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(-214.0710);
  scale.tare();
}

void loop() {
//  float weight = scale.get_units(5);
  Serial.print("oldweight = ");
  Serial.println(oldweight);
  Serial.print("weight = ");
  Serial.println(weight);
  if (weight > 10000.0) {
    weight = scale.get_units(5);
    oldweight = weight;
    digitalWrite(over, HIGH);
    digitalWrite(over2, HIGH);
    Serial.println("over");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Exceeded weight:");
    lcd.setCursor(0, 1);
    lcd.print(weight, 2);
    lcd.print(" g");
    tone(BUZZER_PIN, 1000);
    curMillis = millis();
    if (curMillis - prevMillis >= interval) {
      prevMillis = curMillis;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  } else if (weight < 100.0) {
    weight = scale.get_units(5);
    oldweight = weight;
    digitalWrite(under, HIGH);
    digitalWrite(under2, HIGH);
    Serial.println("under");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Underweight:");
    lcd.setCursor(0, 1);
    lcd.print(weight, 2);
    lcd.print(" g");
    noTone(BUZZER_PIN);
    digitalWrite(LED_PIN, LOW);
//    curMillis = millis();
//    if (curMillis - prevMillis >= interval) {
//      prevMillis = curMillis;
//      ledState = !ledState;
//      digitalWrite(LED_PIN, ledState);
//    }
  } else {
//    if (x=0) {
//      prevMillis2 = millis();
//      x = 1;
//    }
//    curMillis2 = millis();
//    if (curMillis2 - prevMillis2 >= 30000) {
//      digitalWrite(under, HIGH);
//      digitalWrite(under2, HIGH);
//    }
    oldweight = weight;
    digitalWrite(over, LOW);
    digitalWrite(under, LOW);
    digitalWrite(error, LOW);
    digitalWrite(over2, LOW);
    digitalWrite(under2, LOW);
    digitalWrite(error2, LOW);
    Serial.println("normal");
    noTone(BUZZER_PIN);
    digitalWrite(LED_PIN, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Weight:");
    lcd.setCursor(0, 1);
    lcd.print(weight, 2);
    lcd.print(" g");
    weight = scale.get_units(5);
    while (weight - oldweight >250.0 || oldweight - weight > 250.0) {
      weight = scale.get_units(5);
      digitalWrite(error, HIGH);
      digitalWrite(error2, HIGH);
      Serial.print("oldweight = ");
      Serial.println(oldweight);
      Serial.print("weight = ");
      Serial.println(weight);
      Serial.println("error");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Don't change weight:");
      lcd.setCursor(0, 1);
      lcd.print("OldWeight:");
      lcd.print(oldweight, 2);
      lcd.print(" g");
      lcd.setCursor(0, 2);
      lcd.print("Weight:");
      lcd.print(weight, 2);
      lcd.print(" g");
      tone(BUZZER_PIN, 1000);
      curMillis = millis();
      if (curMillis - prevMillis >= interval) {
        prevMillis = curMillis;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
      }
      delay(1000);
    }
  }
  delay(3000);
}
