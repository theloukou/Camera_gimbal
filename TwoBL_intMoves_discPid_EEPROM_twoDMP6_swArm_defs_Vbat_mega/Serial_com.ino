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
    serialSettingsPrint();
  }
  else if (serialStr.startsWith("SET")) {
    pitchSetpoint = serialStr.substring(3, 7).toDouble();
    rollSetpoint = serialStr.substring(8).toDouble();
    PIDzero();
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

    calcK();
  }
}

void serialSettingsPrint() {
#ifndef JSON
  Serial.print(pitchKp); Serial.print("\t");
  Serial.print(pitchKi); Serial.print("\t");
  Serial.print(pitchKd); Serial.print("\t");
  Serial.print(rollKp); Serial.print("\t");
  Serial.print(rollKi); Serial.print("\t");
  Serial.print(rollKd); Serial.print("\t");
  Serial.println(PIDbound);
  //wait 3sec for user to read
  delay(3000);
#endif
#ifdef JSON
  Serial.print("{\"pKp\":");  Serial.print(pitchKp);
  Serial.print(",\"pKi\":");  Serial.print(pitchKi);
  Serial.print(",\"pKd\":");  Serial.print(pitchKd);
  Serial.print(",\"rKp\":");  Serial.print(rollKp);
  Serial.print(",\"rKi\":");  Serial.print(rollKi);
  Serial.print(",\"rKd\":");  Serial.print(rollKd);
  Serial.println("}");
#endif

  //reset FIFOs before continuing
#ifdef CAM_MPU
  camIMU.resetFIFO();
#endif
#ifdef FR_MPU
  frIMU.resetFIFO();
#endif
}

//startup serial commands printout
void serialCommandsPrint() {
  Serial.println("\nAvailable serial commands:");
  Serial.println("\t'arm': Arms motors");
  Serial.println("\t'disarm': Disarms motors");
  Serial.println("\t'SETpppp rrrr': Change current axis setpoints in degrees");
  Serial.println("\t'getSettings': Prints current PID settings, format:");
  Serial.println("\t\tp.kp p.ki p.kd r.kp r.ki r.kd bb as below");
  Serial.println("\t'Sp.kp p.ki p.kd r.kp r.ki r.kd bbE': Change PID settings, where:");
  Serial.println("\t\tp.kp/ki/kd: Pitch Kp/Ki/Kd parameters");
  Serial.println("\t\tr.kp/ki/kd: Roll Kp/Ki/Kd parameters");
  Serial.println("\t\tbb: Maximum PID output value boundary\n");
}

void serialDataPrint() {
#ifndef JSON
  Serial.print(camIMUpitch);
  Serial.print("\t");
  Serial.print(pitchOutput);
  Serial.print("\t");
  Serial.print(frIMUpitch);
  Serial.print("\tP-R\t");
  Serial.print(camIMUroll);
  Serial.print("\t");
  Serial.print(rollOutput);
  Serial.print("\t");
  Serial.println(frIMUroll);
#endif
#ifdef JSON
  Serial.print("{\"pi\":"); Serial.print(camIMUpitch);
  Serial.print(",\"po\":"); Serial.print(pitchOutput);
  Serial.print(",\"ri\":"); Serial.print(camIMUroll);
  Serial.print(",\"ro\":"); Serial.print(rollOutput);
  Serial.println("}");
#endif
}
#endif
