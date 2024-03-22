#include "SoloBrushlessMotor.h"

SoloBrushlessMotor *myMotor;

void setup()
{
  Serial.begin(115200);
  HWSERIAL.begin(115200);
  delay(1000);

  // Motor declaration and setup
  myMotor = new SoloBrushlessMotor(0, HWSERIAL, SoloBrushlessMotor::BaudRate::RATE_115200); // Dynamic is safe here since this exist the whole runtime
  myMotor->init(2.7, 16);
  myMotor->calibration();
}

void loop()
{
  // Set motor current
  myMotor->setCurrent(-0.3);

  myMotor -> getInformation();
  delay(10000);
}
