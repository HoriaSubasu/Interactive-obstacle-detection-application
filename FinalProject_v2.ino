//-----------------libraries--------------------------------

#include <LiquidCrystal.h>
#include <IRremote.h>
#include <EEPROM.h>

//-----------------defining necessities---------------------

#define trigPin 12
#define echoPin 3

#define yellowLed 11
#define redLed 13
#define greenLed 10

#define lcdRsPin A5
#define lcdEPin A3
#define lcdD4Pin 4
#define lcdD5Pin 5
#define lcdD6Pin 6
#define lcdD7Pin 7

#define buttonPin 2

#define photoresistorPin A0

#define irReceivePin A2
#define controllerOkButton 28
#define controller1Button 69
#define controller2Button 70
#define controller3Button 71
#define controller0Button 25
#define controllerStarButton 22
#define controllerDiezButton 13

//-----------------initialising variables-------------------

LiquidCrystal lcd(lcdRsPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);

unsigned long lastTimeUltrasonicTrigger = millis();
unsigned long lastTimeYellowLedBlink = millis();
unsigned long lastTimeLcdClear = millis();
unsigned long lcdClearDelay = 100;
unsigned long ultrasonicTriggerDelay = 60;
unsigned long yellowLedDelay;
unsigned long ledsLoopDelay = 300;

volatile unsigned long pulseInTimeEnd;
volatile unsigned long pulseInTimeBegin;
volatile unsigned long newDistanceAvailable = false;
volatile unsigned long buttonRelease = false;

double previousDistance = 400.0;

int yellowLedState = 0;//  0 - Led is LOW,  1 - Led is HIGH.
int clearDisplay = -1;// for the display clear function so that we dont clear every time we print the distance. 1 - you can clear the screen when the distance is lower than 50 once,  0 - you can clear the screen when the distance is greater than 50 once.
int obstacleFoundWriting = 0;// 0 - we never wrote on the lcd screen 'obstacle found' while in loop , 1 - we did write on the lcd screen 'obstacle found' while in loop. We use this so that we dont write the same thing while we are in the loop.
int controllerCommand;// stores the values from the ir.
int lcd2Clear = 0;// for the display clear function so that we dont clear every time we enter this case. 0 - we haven t clear,    1 - we have cleared once.
int lcd3Clear = 0;// for the display clear function so that we dont clear every time we enter this case. 0 - we haven t clear,    1 - we have cleared once.

//------------------functions------------------------------

void triggerUltrasonicSensor() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

void echoPinInterrupt() {
  if (digitalRead(echoPin) == HIGH) {
    pulseInTimeBegin = micros();
  }
  else {
    pulseInTimeEnd = micros();
    newDistanceAvailable = true;
  }
}

void buttonPinInterrupt() {
  buttonRelease = true;
}

double getUltrasonicDistance() {
  double durationMicros = pulseInTimeEnd - pulseInTimeBegin;
  double distance = durationMicros / 58.0;
  if (distance > 400.0) {
    return previousDistance;
  }
  else {
    previousDistance = distance;
    return previousDistance;
  }
}

void lcdDisplayInCm() {
  if (getUltrasonicDistance() > 50) {
    if (clearDisplay != 1) {
      clearDisplay = 1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Dist: ");
      lcd.print(getUltrasonicDistance());
      lcd.print("cm");
      lcd.setCursor(0, 1);
      lcd.print("No obstacle.");
    }
    else {
      lcd.setCursor(6, 0);
      lcd.print(getUltrasonicDistance());
      lcd.print("cm");
    }
  }
  else if (getUltrasonicDistance() < 50 && getUltrasonicDistance() > 10) {
    if (clearDisplay != 0) {
      clearDisplay = 0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Dist: ");
      lcd.print(getUltrasonicDistance());
      lcd.print("cm");
      lcd.setCursor(0, 1);
      lcd.print("!! Warning !!");
    }
    else {
      lcd.setCursor(6, 0);
      lcd.print(getUltrasonicDistance());
      lcd.print("cm");
    }
  }
  else if (getUltrasonicDistance() < 10) {
    lcd.clear();
    while (buttonRelease == false && controllerCommand != controllerOkButton) {
      unsigned long timeNow = millis();
      if (obstacleFoundWriting == 0) {
        lcd.setCursor(0, 0);
        lcd.print("!!! Obstacle !!!");
        lcd.setCursor(0, 1);
        lcd.print("Press to unlock");
      }
      obstacleFoundWriting = 1;
      if (timeNow - lastTimeYellowLedBlink > ledsLoopDelay) {
        lastTimeYellowLedBlink += ledsLoopDelay;
        if (yellowLedState == 0) {
          digitalWrite(yellowLed, HIGH);
          digitalWrite(redLed, HIGH);
          yellowLedState = 1;
        }
        else {
          digitalWrite(yellowLed, LOW);
          digitalWrite(redLed, LOW);
          yellowLedState = 0;
        }
      }
      makeTheGreenLedBlink();
      if (IrReceiver.decode()) {
        IrReceiver.resume();
        controllerCommand = IrReceiver.decodedIRData.command;
      }
    }
    buttonRelease = false;
    obstacleFoundWriting = 0;
    clearDisplay = -1;
    controllerCommand = -1;
    digitalWrite(redLed, LOW);
  }
}

void lcdDisplayInInch() {
  if ((getUltrasonicDistance() * 0.393) > 19.65) {
    if (clearDisplay != 1) {
      clearDisplay = 1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Dist: ");
      lcd.print(getUltrasonicDistance() * 0.393);
      lcd.print("inch");
      lcd.setCursor(0, 1);
      lcd.print("No obstacle.");
    }
    else {
      lcd.setCursor(6, 0);
      lcd.print(getUltrasonicDistance() * 0.393);
      lcd.print("inch");
    }
  }
  else if ((getUltrasonicDistance() * 0.393) < 19.65 && (getUltrasonicDistance() * 0.393) > 3.93) {
    if (clearDisplay != 0) {
      clearDisplay = 0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Dist: ");
      lcd.print(getUltrasonicDistance() * 0.393);
      lcd.print("inch");
      lcd.setCursor(0, 1);
      lcd.print("!! Warning !!");
    }
    else {
      lcd.setCursor(6, 0);
      lcd.print(getUltrasonicDistance() * 0.393);
      lcd.print("inch");
    }
  }
  else if ((getUltrasonicDistance() * 0.393) < 3.93) {
    lcd.clear();
    while (buttonRelease == false && controllerCommand != controllerOkButton) {
      unsigned long timeNow = millis();
      if (obstacleFoundWriting == 0) {
        lcd.setCursor(0, 0);
        lcd.print("!!! Obstacle !!!");
        lcd.setCursor(0, 1);
        lcd.print("Press to unlock");
      }
      obstacleFoundWriting = 1;
      if (timeNow - lastTimeYellowLedBlink > ledsLoopDelay) {
        lastTimeYellowLedBlink += ledsLoopDelay;
        if (yellowLedState == 0) {
          digitalWrite(yellowLed, HIGH);
          digitalWrite(redLed, HIGH);
          yellowLedState = 1;
        }
        else {
          digitalWrite(yellowLed, LOW);
          digitalWrite(redLed, LOW);
          yellowLedState = 0;
        }
      }
      makeTheGreenLedBlink();
      if (IrReceiver.decode()) {
        IrReceiver.resume();
        controllerCommand = IrReceiver.decodedIRData.command;
      }
    }
    buttonRelease = false;
    obstacleFoundWriting = 0;
    clearDisplay = -1;
    controllerCommand = -1;
    digitalWrite(redLed, LOW);
  }
}

void makeTheYellowLedBlink(unsigned long timeNow) {
  yellowLedDelay = 4 * getUltrasonicDistance();
  if (timeNow - lastTimeYellowLedBlink > yellowLedDelay) {
    lastTimeYellowLedBlink += yellowLedDelay;
    if (yellowLedState == 0) {
      digitalWrite(yellowLed, HIGH);
      yellowLedState = 1;
    }
    else {
      digitalWrite(yellowLed, LOW);
      yellowLedState = 0;
    }
  }
}

void makeTheGreenLedBlink() {
  analogWrite(greenLed, 255 - (analogRead(photoresistorPin) / 4));
}

//----------------------the setup------------------------

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.clear();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(photoresistorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(echoPin), echoPinInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonPinInterrupt, FALLING);
  IrReceiver.begin(irReceivePin);
  digitalWrite(yellowLed, LOW);
  EEPROM.write(0, controller1Button);
  controllerCommand = EEPROM.read(1);
}

//--------------------the main code----------------------

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long timeNow = millis();
  if (timeNow - lastTimeUltrasonicTrigger > ultrasonicTriggerDelay) {
    lastTimeUltrasonicTrigger += ultrasonicTriggerDelay;
    triggerUltrasonicSensor();
  }
  if (IrReceiver.decode()) {
    IrReceiver.resume();
    if (IrReceiver.decodedIRData.command != 0) {
      controllerCommand = IrReceiver.decodedIRData.command;
    }
  }
  switch (controllerCommand) {
    case controller1Button: {
        if (newDistanceAvailable == true) {
          newDistanceAvailable = false;
          lcdDisplayInCm();
        }
        lcd2Clear = 0;
        lcd3Clear = 0;
        break;
      }
    case controller2Button: {
        if (lcd2Clear == 0) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Luminosity: ");
          lcd.print(analogRead(photoresistorPin));
          lcd2Clear = 1;
        }
        else {
          lcd.setCursor(12, 0);
          lcd.print(analogRead(photoresistorPin));
        }
        clearDisplay = -1;
        lcd3Clear = 0;
        break;
      }
    case controller3Button: {
        if (lcd3Clear == 0) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Press */# to");
          lcd.setCursor(0, 1);
          lcd.print("reset settings.");
          lcd3Clear = 1;
        }
        clearDisplay = -1;
        lcd2Clear = 0;
        break;
      }
    case controller0Button: {
        clearDisplay = -1;
        if (newDistanceAvailable == true) {
          newDistanceAvailable = false;
          lcdDisplayInInch();
        }
        lcd2Clear = 0;
        lcd3Clear = 0;
        break;
      }
    case controllerStarButton: {
        if (EEPROM.read(1) == controller3Button) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Settings have");
          lcd.setCursor(0, 1);
          lcd.print("been reset");
          delay(2000);
          controllerCommand = EEPROM.read(0);
        }
        else{
          controllerCommand = EEPROM.read(1);
        }
        break;
      }
    case controllerDiezButton: {
        if (EEPROM.read(1) == controller3Button) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Settings have");
          lcd.setCursor(0, 1);
          lcd.print("been reset");
          delay(2000);
          controllerCommand = EEPROM.read(0);
        }
        else{
          controllerCommand = EEPROM.read(1);
        }
        break;
      }
    default: {
        if (newDistanceAvailable == true) {
          newDistanceAvailable = false;
          lcdDisplayInCm();
        }
      }
  }
  makeTheYellowLedBlink(timeNow);
  makeTheGreenLedBlink();
  if (EEPROM.read(1) != controllerCommand && controllerCommand != controllerStarButton && controllerCommand && controllerDiezButton) {
    EEPROM.write(1, controllerCommand);
  }

}
