#ifdef CAM_MPU
//Camera IMU first setup
void cameraIMUsetup() {
  // initialize device
  camIMU.initialize();

  // verify connection
  Serial.println(camIMU.testConnection() ? F("camIMU connection successful") : F("camIMU connection failed"));

  // load and configure the DMP
  camIMUdevStatus = camIMU.dmpInitialize();

#ifdef NO_CALIBRATION
  //set Camera IMU offsets
  camIMU.setXAccelOffset(6);
  camIMU.setYAccelOffset(-705);
  camIMU.setZAccelOffset(1162);
  camIMU.setXGyroOffset(-16);
  camIMU.setYGyroOffset(-66);
  camIMU.setZGyroOffset(16);
#endif

  // make sure it worked (returns 0 if so)
  if (camIMUdevStatus == 0) {
#ifndef NO_CALIBRATION
    // Calibration Time: generate offsets and calibrate our MPU6050
    Serial.println("Calibrating");
    delay(1500);
    camIMU.CalibrateAccel(6);
    camIMU.CalibrateGyro(6);
    camIMU.PrintActiveOffsets();
#endif
    // turn on the DMP, now that it's ready
    camIMU.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("camIMU DMP ready!"));
    camIMUReady = true;

    // get expected DMP packet size for later comparison
    camIMUpacketSize = camIMU.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("camIMU DMP Initialization failed (code "));
    Serial.print(camIMUdevStatus);
    Serial.println(F(")"));
  }
}

//get camera IMU data, calculate needed angles
void cameraIMUdata() {
  // get current camIMU FIFO count
  camIMUfifoCount = camIMU.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if (camIMUfifoCount >= 1024) {
    // reset so we can continue cleanly
    camIMU.resetFIFO();
    Serial.println(F("camIMU FIFO overflow!"));
  }
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (camIMUfifoCount >= 42) {
    // read a packet from FIFO
    camIMU.getFIFOBytes(camIMUfifoBuffer, camIMUpacketSize);

    // calculate Euler angles in degrees
    camIMU.dmpGetQuaternion(&camIMUq, camIMUfifoBuffer);
    camIMU.dmpGetGravity(&camIMUgravity, &camIMUq);
    camIMU.dmpGetYawPitchRoll(camIMUypr, &camIMUq, &camIMUgravity);
  }

  //get desired used angles
  camIMUpitch = -camIMUypr[2] * 180 / M_PI;
  camIMUroll = -camIMUypr[1] * 180 / M_PI;
}
#endif

#ifdef FR_MPU
//Frame IMU first setup
void frameIMUsetup() {
  // initialize device
  frIMU.initialize();

  // verify connection
  Serial.println(frIMU.testConnection() ? F("frIMU connection successful") : F("frIMU connection failed"));

  // load and configure the DMP
  frIMUdevStatus = frIMU.dmpInitialize();

  //set Frame IMU offsets
  //  frIMU.setXAccelOffset(291);
  //  frIMU.setYAccelOffset(1282);
  //  frIMU.setZAccelOffset(1005);
  //  frIMU.setXGyroOffset(107);
  //  frIMU.setYGyroOffset(-31);
  //  frIMU.setZGyroOffset(12);

  // make sure it worked (returns 0 if so)
  if (frIMUdevStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    Serial.println("Calibrating");
    delay(1500);
    frIMU.CalibrateAccel(6);
    frIMU.CalibrateGyro(6);
    frIMU.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    frIMU.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("frIMU DMP ready!"));
    frIMUReady = true;

    // get expected DMP packet size for later comparison
    frIMUpacketSize = frIMU.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("frIMU DMP Initialization failed (code "));
    Serial.print(frIMUdevStatus);
    Serial.println(F(")"));
  }
}

void frameIMUdata() {
  // get current frIMU FIFO count
  frIMUfifoCount = frIMU.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if (frIMUfifoCount >= 1024) {
    // reset so we can continue cleanly
    frIMU.resetFIFO();
    Serial.println(F("frIMU FIFO overflow!"));
  }
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (frIMUfifoCount >= 42) {
    // read a packet from FIFO
    frIMU.getFIFOBytes(frIMUfifoBuffer, frIMUpacketSize);

    // calculate Euler angles in degrees
    frIMU.dmpGetQuaternion(&frIMUq, frIMUfifoBuffer);
    frIMU.dmpGetGravity(&frIMUgravity, &frIMUq);
    frIMU.dmpGetYawPitchRoll(frIMUypr, &frIMUq, &frIMUgravity);
  }

  //get desired used angles
  frIMUpitch = -frIMUypr[2] * 180 / M_PI;
  frIMUroll = -frIMUypr[1] * 180 / M_PI;
}
#endif
