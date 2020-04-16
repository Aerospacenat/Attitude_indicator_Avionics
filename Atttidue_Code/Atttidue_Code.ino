#include <Wire.h> //Sensor
#include <MPU6050.h> //Sensor
MPU6050 mpu; //Sensor -  MU-6050 SDA to SDA and SCL TO SCL with a VCC and GND attached

#include <LiquidCrystal.h>
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

#include <LedControl.h>
const int DIN_PIN = 6;
const int CS_PIN =  5;
const int CLK_PIN = 4;
const uint64_t Warn_right[] = { 0x80a4949c949c8080 }; // right
const uint64_t Warn_left[] =  { 0x0125151d151d0101 }; //left
const uint64_t Warn_up[] =    { 0x000004041c141cff }; //up
const uint64_t Warn_down[] =  { 0xff0004041c141c00 }; //down 
const uint64_t roll_Mode[] =  { 0x0024141c141c0000 }; //Roll mode  
const uint64_t roll_1[] =     { 0x0024141c141c0002 }; //7.5Degree 
const uint64_t roll_2[] =     { 0x0024141c141c0006 }; //15D 
const uint64_t roll_3[] =     { 0x0024141c141c000e }; //22.5D 
const uint64_t roll_4[] =     { 0x0024141c141c001e }; //30D 
const uint64_t roll_5[] =     { 0x0024141c141c003e }; //37.5D 
const uint64_t roll_6[] =     { 0x0024141c141c007e }; //45D 
const uint64_t pitch_Mode[] = { 0x000004041c141c00 }; //Pitch mode  
const uint64_t pitch_1[] =    { 0x000004041c141d00 }; //5Degree
const uint64_t pitch_2[] =    { 0x000004041c151d00 }; //10D 
const uint64_t pitch_3[] =    { 0x000004041d151d00 }; //15D 
const uint64_t pitch_4[] =    { 0x000004051d151d00 }; //20D 
const uint64_t pitch_5[] =    { 0x000005051d151d00 }; //25D 
const uint64_t pitch_6[]      { 0x000105051d151d00 }; //30D 
LedControl display = LedControl(DIN_PIN, CLK_PIN, CS_PIN);
int i = 0;

//Button
int buttonPin = 3;
int lastButtonState = 0;

int ledPins[] = {A8,A9,A10,A11,A12,A13,A14,A15}; // pins A14 and A15 are used to indicate warnings

void setup() {
  for(int x = 0; x < 8; x++) pinMode(ledPins[x],OUTPUT); // setting up the LED pins
  pinMode(buttonPin,INPUT); // button pin set up 
  display.clearDisplay(0); // LED dot matrix setup 
  display.shutdown(0, false); // LED dot matrix setup 
  display.setIntensity(0, 10); // LED dot matrix setup 
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) { //setting up the sensor aswell as checking that it is connected
    lcd.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  lcd.begin(16, 2); //16 characters on 2 lines and setting up the LCD
}

void displayImage(uint64_t image) { // LED Dot matix, setting each dot in the matrix depending on the code that was called
  for (int i = 0; i < 8; i++) {
    byte row = (image >> i * 8) & 0xFF;
    for (int j = 0; j < 8; j++) {
      display.setLed(0, i, j, bitRead(row, j));
      }
   }
}

void loop() {
  static byte lastState = HIGH; // variable to remember the last state of the button
  static byte selectedMode = 1; // variable to remember selected mode
  byte currState = digitalRead(buttonPin); // read the button
  if (currState != lastState) { // if it changed
    if (lastState == LOW) {  // button was released if last state was LOW
      if (selectedMode == 1) // change selected mode
        selectedMode = 2;
      else
        selectedMode = 1;
      lastButtonState = 1; // reset currentState so we start from the beginning
      } 
    lastState = currState;  // remember last state
  }
  if (selectedMode == 1) {
    pitchMode(); // goes to pitch mode
  }
  else {
    rollMode(); // goes to roll mode
  }
}

void pitchMode() { 
  // Pitch values
  Vector normAccel = mpu.readNormalizeAccel(); //reads the Accellorometer values 
  float pitch = 0; //float 
  pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI; //https://www.electronicwings.com/arduino/mpu6050-interfacing-with-arduino-uno
  pitch = lround(pitch); //rounds the pitch value to the nearest digit rather than having decimals
  lcd.setCursor(3, 0); // set in to remove some of the decimal places
  lcd.print(" Pitch = ");
  int pitchNew = (int)pitch; // float to int conversion
  if(pitchNew % 2 == 0){ // https://forum.arduino.cc/index.php?topic=525358.0
    lcd.print(pitch);
    }

  displayImage(pitch_Mode[i]);

  if (pitch >= 5 || pitch <= -5){ // if the pitch is over 5 or -5 shows the LED DOT matrix
    displayImage(pitch_1[i]);
  }
  if (pitch >= 10 || pitch <= -10){ 
    displayImage(pitch_2[i]);
  }
  if (pitch >= 15 || pitch <= -15){  
    displayImage(pitch_3[i]);
  }
  if (pitch >= 20 || pitch <= -20){ 
    displayImage(pitch_4[i]);
  }
  if (pitch >= 25 || pitch <= -25){  
    displayImage(pitch_5[i]);
  }
  if (pitch >= 30 || pitch <= -30){  
    displayImage(pitch_6[i]);
  }

  float roll = 0; // this section is for the LED lights to show roll whilst on pitch mode
  roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI; // this works out the roll
  roll = round(roll);
  if (roll >= 7.5 || roll <= -7.5){ // if the roll is over 7.5 or -7.5 set LED to ON
    digitalWrite(ledPins[0], HIGH);
  }
  if (roll >= 15 || roll <= -15){
    digitalWrite(ledPins[1], HIGH);
  }
  if (roll >= 22.5 || roll <= -22.5){
    digitalWrite(ledPins[2], HIGH);
  }
  if (roll >= 30 || roll <= -30){
    digitalWrite(ledPins[3], HIGH);
  }
  if (roll >= 37.5 || roll <= -37.5){
    digitalWrite(ledPins[4], HIGH);
  }
  if (roll >= 45 || roll <= -45){
    digitalWrite(ledPins[5], HIGH);
  }
  if (roll >= 45) {
    digitalWrite(ledPins[6], HIGH);
    }
  if (roll <= -45) {
    digitalWrite(ledPins[7], HIGH);
    }
  delay(40);
  for(int x = 0; x < 8; x++)(digitalWrite(ledPins[x], LOW)); // set all LEDS to off

  if (pitch >= 30) { //warning for if the pitch is too high
    lcd.setCursor(0, 1); //1st 0 is amount of characters, for middle do lcd.setCursor(8,0)  2nd is the line selected
    lcd.write(" Pitch over 30"); 
    displayImage(Warn_up[i]);
    }
  else if (pitch <= -30) { // if else is used as it is more efficent as it stops doing comparisons as soon as it finds one is true. if-if-if does every comparison.
    lcd.setCursor(0, 1);
    lcd.write("Pitch below -30"); //if pitch is over or equal to 40 warning is shown
    displayImage(Warn_down[i]);
    }
  else {
    lcd.setCursor(0, 1);
    lcd.write("No warning       "); // this is 16 characters long to "clear" the line. Because using lcd.clear would not be appropriate as it will be blank throught the program
    }
  delay(210); //4times a second 250 - any other delays 
}

void rollMode() {
  Vector normAccel = mpu.readNormalizeAccel();
  float roll = 0;
  roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI; // formula to work out the roll
  roll = round(roll); // round the roll to digit removing the decimals
  lcd.setCursor(4, 0); // set in to removed decimal places and to be in the same position  as Pitch
  lcd.write(" Roll = ");
  int rollNew = (int)roll; //Float to Int conversion
  if(rollNew % 2 == 0){
    lcd.print(roll);
  }
  
  displayImage(roll_Mode[i]);

  if (roll >= 7.5 || roll <= -7.5){ // if roll is over 7.5 or -7.5 shows the LED DOT matrix
    displayImage(roll_1[i]);
  }
  if (roll >= 15 || roll <= -15){
    displayImage(roll_2[i]);
  }
  if (roll >= 22.5 || roll <= -22.5){
    displayImage(roll_3[i]);
  }
  if (roll >= 30 || roll <= -30){
    displayImage(roll_4[i]);
  }
  if (roll >= 37.5 || roll <= -37.5){
    displayImage(roll_5[i]);
  }
  if (roll >= 45 || roll <= -45){
    displayImage(roll_6[i]);
  }

  float pitch = 0; //float  this section is for the LED lights to show pitch whilst on roll mode
  pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  if (pitch > 5 || pitch < -5){ // if the pitch is over 5 or -5 shows the LED comes on
    digitalWrite(ledPins[0], HIGH);
  }
  if (pitch >= 10 || pitch <= -10){
    digitalWrite(ledPins[1], HIGH);
  }
  if (pitch >= 15 || pitch <= -15){
    digitalWrite(ledPins[2], HIGH);
  }
  if (pitch >= 20 || pitch <= -20){
    digitalWrite(ledPins[3], HIGH);
  }
  if (pitch >= 25 || pitch <= -25){
    digitalWrite(ledPins[4], HIGH);
  }
  if (pitch >= 30 || pitch <= -30){
    digitalWrite(ledPins[5], HIGH);
  }
  if (pitch >= 30) {
    digitalWrite(ledPins[6], HIGH);
    }
  if (pitch <= -30) {
    digitalWrite(ledPins[7], HIGH);
    }
  delay(40);
  for(int x = 0; x < 8; x++)(digitalWrite(ledPins[x], LOW)); // resets all the LED's
  
  if (roll >= 45) { // if roll is over or equal to 45 warning is shown
    lcd.setCursor(0, 1); //1st 0 is amount of char, for middle do lcd.setCursor(8,0)
    lcd.write("Roll over 45"); 
    displayImage(Warn_right[i]); 
    }
  else if (roll <= -45) {
    lcd.setCursor(0, 1);
    lcd.write("Roll over -45");
    displayImage(Warn_left[i]);
    }
  else {
    lcd.setCursor(0, 1);
    lcd.write("No warning       ");  
    }
  delay(210); //4times a second 250 - any other delays 
}
