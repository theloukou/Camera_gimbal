//255 PWM sine table
//const byte matrix[] = {128, 131, 134, 137, 140, 143, 146, 149, 153, 156, 159, 162, 165, 168, 171, 174, 177, 180, 182,
//                       185, 188, 191, 194, 196, 199, 201, 204, 206, 209, 211, 214, 216, 218, 220, 223, 225, 227, 229,
//                       230, 232, 234, 236, 237, 239, 240, 242, 243, 245, 246, 247, 248, 249, 250, 251, 251, 252, 253,
//                       253, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 253, 253, 252, 251, 251, 250,
//                       249, 248, 247, 246, 245, 243, 242, 240, 239, 237, 236, 234, 232, 230, 229, 227, 225, 223, 220,
//                       218, 216, 214, 211, 209, 206, 204, 201, 199, 196, 194, 191, 188, 185, 182, 180, 177, 174, 171,
//                       168, 165, 162, 159, 156, 153, 149, 146, 143, 140, 137, 134, 131, 128, 124, 121, 118, 115, 112,
//                       109, 106, 102, 99, 96, 93, 90, 87, 84, 81, 78, 75, 73, 70, 67, 64, 61, 59, 56,
//                       54, 51, 49, 46, 44, 41, 39, 37, 35, 32, 30, 28, 26, 25, 23, 21, 19, 18, 16,
//                       15, 13, 12, 10, 9, 8, 7, 6, 5, 4, 4, 3, 2, 2, 1, 1, 0, 0, 0,
//                       0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 4, 5, 6, 7, 8, 9, 10, 12,
//                       13, 15, 16, 18, 19, 21, 23, 25, 26, 28, 30, 32, 35, 37, 39, 41, 44, 46, 49,
//                       51, 54, 56, 59, 61, 64, 67, 70, 73, 75, 78, 81, 84, 87, 90, 93, 96, 99, 102,
//                       106, 109, 112, 115, 118, 121, 124, 127
//                      };

//360 PWM sine table
const byte matrix[] = {128, 130, 132, 134, 136, 139, 141, 143, 145, 148, 150, 152, 154, 156, 158, 161, 163, 165, 167, 169, 171,
                       173, 175, 177, 179, 182, 184, 186, 188, 189, 191, 193, 195, 197, 199, 201, 203, 204, 206, 208, 210, 211,
                       213, 215, 216, 218, 219, 221, 222, 224, 225, 227, 228, 230, 231, 232, 233, 235, 236, 237, 238, 239, 240,
                       241, 242, 243, 244, 245, 246, 247, 247, 248, 249, 250, 250, 251, 251, 252, 252, 253, 253, 254, 254, 254,
                       254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 254, 253, 253, 253, 252, 252, 251,
                       250, 250, 249, 249, 248, 247, 246, 245, 245, 244, 243, 242, 241, 240, 239, 238, 236, 235, 234, 233, 231,
                       230, 229, 227, 226, 225, 223, 222, 220, 219, 217, 215, 214, 212, 210, 209, 207, 205, 204, 202, 200, 198,
                       196, 194, 192, 190, 188, 187, 185, 183, 181, 178, 176, 174, 172, 170, 168, 166, 164, 162, 160, 157, 155,
                       153, 151, 149, 146, 144, 142, 140, 138, 135, 133, 131, 129, 126, 124, 122, 120, 117, 115, 113, 111, 109,
                       106, 104, 102, 100, 98, 95, 93, 91, 89, 87, 85, 83, 81, 79, 77, 74, 72, 70, 68, 67, 65,
                       63, 61, 59, 57, 55, 53, 51, 50, 48, 46, 45, 43, 41, 40, 38, 36, 35, 33, 32, 30, 29,
                       28, 26, 25, 24, 22, 21, 20, 19, 17, 16, 15, 14, 13, 12, 11, 10, 10, 9, 8, 7, 6,
                       6, 5, 5, 4, 3, 3, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                       0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 7, 8, 8, 9, 10, 11,
                       12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25, 27, 28, 30, 31, 33, 34, 36, 37,
                       39, 40, 42, 44, 45, 47, 49, 51, 52, 54, 56, 58, 60, 62, 64, 66, 67, 69, 71, 73, 76,
                       78, 80, 82, 84, 86, 88, 90, 92, 94, 97, 99, 101, 103, 105,  107, 110, 112, 114, 116, 119, 121,
                       123, 125, 127
                      };

byte maxPwm = 255;

//initialize motor phase startings
volatile int pitchOnePos = 0 + STARTINGOFFSET;
volatile int pitchTwoPos = (STEPS / 3) + STARTINGOFFSET;
volatile int pitchThreePos = ((STEPS / 3) * 2) + STARTINGOFFSET;
volatile int rollOnePos = 0 + STARTINGOFFSET;
volatile int rollTwoPos = (STEPS / 3) + STARTINGOFFSET;
volatile int rollThreePos = ((STEPS / 3) * 2) + STARTINGOFFSET;

//arm switch interrupt routine
void switched() {
  if (!digitalRead(ARMSW)) {
    disarm();
  }
  else {
    arm();
  }
}

// motor arm routine
void arm() {
  digitalWrite(ENPITCH, 1);
  digitalWrite(ENROLL, 1);
  Timer1.attachInterrupt(rollPos);
  Timer5.attachInterrupt(pitchPos);
}

// motor disarm routine
void disarm() {
  digitalWrite(ENPITCH, 0);
  digitalWrite(ENROLL, 0);
  Timer1.detachInterrupt();
  Timer5.detachInterrupt();
}

//PID output decoders
void rollOutDecode(double output) {
  if (signbit(output)) rollDir = -1;
  else rollDir = 1;;

  rollIntPeriod = 1 / (abs(output)) * 100000;
  if (rollIntPeriod == 0) rollIntPeriod = 1000000;
  Timer1.setPeriod(rollIntPeriod);
}

void pitchOutDecode(double output) {
  if (signbit(output)) pitchDir = -1;
  else pitchDir = 1;

  pitchIntPeriod = 1 / (abs(output)) * 100000;
  if (pitchIntPeriod == 0) pitchIntPeriod = 1000000;
  Timer5.setPeriod(pitchIntPeriod);
}

//roll motor drive
void rollPos() {
  rollOnePos += rollDir;
  rollTwoPos += rollDir;
  rollThreePos += rollDir;

  if (rollOnePos >= STEPS) {
    rollOnePos -= STEPS;
  }
  else if (rollOnePos <= -1) {
    rollOnePos += STEPS;
  }
  if (rollTwoPos >= STEPS) {
    rollTwoPos -= STEPS;
  }
  else if (rollTwoPos <= -1) {
    rollTwoPos += STEPS;
  }
  if (rollThreePos >= STEPS) {
    rollThreePos -= STEPS;
  }
  else if (rollThreePos <= -1) {
    rollThreePos += STEPS;
  }

  analogWrite(ROLLOUT1, matrix[rollOnePos]);
  analogWrite(ROLLOUT2, matrix[rollTwoPos]);
  analogWrite(ROLLOUT3, matrix[rollThreePos]);
}

//pitch motor drive
void pitchPos() {
  pitchOnePos += pitchDir;
  pitchTwoPos += pitchDir;
  pitchThreePos += pitchDir;

  if (pitchOnePos >= STEPS) {
    pitchOnePos -= STEPS;
  }
  else if (pitchOnePos <= -1) {
    pitchOnePos += STEPS;
  }
  if (pitchTwoPos >= STEPS) {
    pitchTwoPos -= STEPS;
  }
  else if (pitchTwoPos <= -1) {
    pitchTwoPos += STEPS;
  }
  if (pitchThreePos >= STEPS) {
    pitchThreePos -= STEPS;
  }
  else if (pitchThreePos <= -1) {
    pitchThreePos += STEPS;
  }

  analogWrite(PITCHOUT1, matrix[pitchOnePos]);
  analogWrite(PITCHOUT2, matrix[pitchTwoPos]);
  analogWrite(PITCHOUT3, matrix[pitchThreePos]);
}
