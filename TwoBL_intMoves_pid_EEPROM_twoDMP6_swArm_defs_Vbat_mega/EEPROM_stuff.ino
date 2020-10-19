//get PID settings form EEPROM
void eeprom_get() {
  int eeAddress = 0;
  EEPROM.get(eeAddress, pitchKp);
  eeAddress += sizeof(double);
  EEPROM.get(eeAddress, pitchKi);
  eeAddress += sizeof(double);
  EEPROM.get(eeAddress, pitchKd);
  eeAddress += sizeof(double);
  EEPROM.get(eeAddress, rollKp);
  eeAddress += sizeof(double);
  EEPROM.get(eeAddress, rollKi);
  eeAddress += sizeof(double);
  EEPROM.get(eeAddress, rollKd);
  eeAddress += sizeof(double);
  EEPROM.get(eeAddress, PIDbound);
}

//update PID settings to EEPROM
void eeprom_put() {
  int eeAddress = 0;
  EEPROM.put(eeAddress, pitchKp);
  eeAddress += sizeof(double);
  EEPROM.put(eeAddress, pitchKi);
  eeAddress += sizeof(double);
  EEPROM.put(eeAddress, pitchKd);
  eeAddress += sizeof(double);
  EEPROM.put(eeAddress, rollKp);
  eeAddress += sizeof(double);
  EEPROM.put(eeAddress, rollKi);
  eeAddress += sizeof(double);
  EEPROM.put(eeAddress, rollKd);
  eeAddress += sizeof(double);
  EEPROM.put(eeAddress, PIDbound);
}
