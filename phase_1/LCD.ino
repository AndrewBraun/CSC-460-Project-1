// LCD libraries and code from http://arduinoinfo.mywikis.net/wiki/LCD-Blue-I2C

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

void lcd_setup() {
  lcd.begin(20,4);
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("Resistor:");
  lcd.setCursor(0,1);
  lcd.print("Stick x axis:");
  lcd.setCursor(0,2);
  lcd.print("Stick y axis:");
}

/*
 *Takes as input:
 *resistance of photoresistor
 *joystick x axis value
 *joystick y axis value
  */
void write_to_lcd(int light_resistance, int joystick_x_value, int joystick_y_value) {

  lcd.setCursor(9,0);
  lcd.print(String(light_resistance) + " Ohm");

  lcd.setCursor(13,1);
  lcd.print(joystick_x_value);

  lcd.setCursor(13,2);
  lcd.print(joystick_y_value);
}
