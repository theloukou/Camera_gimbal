#ifdef SERIAL_ENABLED
//local Serial vars
unsigned long serTime;

//incoming serial data routine
void serialEvent() {
  //reset serial input string
  serialStr = "";

  //wait a bit for whole command to come through
  serTime = millis();
  while (millis() - serTime < 100) {}   //100 msec

  //read serial command string
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    serialStr += inChar;
  }

  //check serial command string against known commands
  if (serialStr == "arm") {
    arm();
  }
  else if (serialStr == "disarm") {
    disarm();
  }
  else if (serialStr == "getSettings") {
    Serial.print(pitchKp); Serial.print("\t");
    Serial.print(pitchKi); Serial.print("\t");
    Serial.print(pitchKd); Serial.print("\t");
    Serial.print(rollKp); Serial.print("\t");
    Serial.print(rollKi); Serial.print("\t");
    Serial.print(rollKd); Serial.print("\t");
    Serial.println(PIDbound);

    //wait 3sec for user to read, reset FIFOs before continuing
    delay(3000);
#ifdef CAM_MPU
    camIMU.resetFIFO();
#endif
#ifdef FR_MPU
    frIMU.resetFIFO();
#endif
  }
  else if ((serialStr.startsWith("S")) && (serialStr.endsWith("E"))) {
    pitchKp = serialStr.substring(1, 5).toDouble();
    pitchKi = serialStr.substring(6, 10).toDouble();
    pitchKd = serialStr.substring(11, 15).toDouble();
    rollKp = serialStr.substring(16, 20).toDouble();
    rollKi = serialStr.substring(21, 25).toDouble();
    rollKd = serialStr.substring(26, 30).toDouble();
    PIDbound = serialStr.substring(31).toDouble();
    eeprom_put();
  }
}

//startup serial commands printout
void serialCommandsPrint() {
  Serial.println("\nAvailable serial commands:");
  Serial.println("\t'arm': Arms motors");
  Serial.println("\t'disarm': Disarms motors");
  Serial.println("\t'getSettings': Prints current PID settings, format:");
  Serial.println("\t\tp.kp p.ki p.kd r.kp r.ki r.kd bb as below");
  Serial.println("\t'Sp.kp p.ki p.kd r.kp r.ki r.kd bbE': Change PID settings, where:");
  Serial.println("\t\tp.kp/ki/kd: Pitch Kp/Ki/Kd parameters");
  Serial.println("\t\tr.kp/ki/kd: Roll Kp/Ki/Kd parameters");
  Serial.println("\t\tbb: Maximum PID output value boundary\n");
}
#endif
