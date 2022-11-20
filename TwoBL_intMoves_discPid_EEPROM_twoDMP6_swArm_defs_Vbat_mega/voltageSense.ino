//local voltage vars
int Vbat;

//voltage sensing routine
void voltage() {
  Vbat = (analogRead(A8) * 4.882) * 3; //get voltage in millivolts
#ifdef VBATSERIAL
  Serial.print("{\"vbat\":"); Serial.print(Vbat);
  Serial.println("}");
#endif

  //low bat check, triggered if Vbat < 9.6Volts
  if (Vbat <= 9600) {
    LedBlinkTime = 10;  //blink LED faster to let user know that Vbat=LOW

    //disable arm switch, disable motors
    detachInterrupt(digitalPinToInterrupt(ARMSW));
    disarm();
  }
}
