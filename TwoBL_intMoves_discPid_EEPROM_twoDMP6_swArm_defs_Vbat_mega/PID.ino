//local PID vars
double pitchOutputDelta, pitchError = 0, pitchErrorPrev1 = 0, pitchErrorPrev2 = 0;
double rollOutputDelta, rollError = 0, rollErrorPrev1 = 0, rollErrorPrev2 = 0;
double pitchK1, pitchK2, pitchK3;
double rollK1, rollK2, rollK3;

//PID calculation
void PIDcalc(double Pinput, double Psetpoint, double *Poutput, double Rinput, double Rsetpoint, double *Routput) {

  pitchErrorPrev2 = pitchErrorPrev1;
  pitchErrorPrev1 = pitchError;
  pitchError = Psetpoint - Pinput;
  pitchOutputDelta = pitchK1 * pitchError + pitchK2 * pitchErrorPrev1 + pitchK3 * pitchErrorPrev2;
  *Poutput = constrain(*Poutput - pitchOutputDelta, -PIDbound, PIDbound);

  rollErrorPrev2 = rollErrorPrev1;
  rollErrorPrev1 = rollError;
  rollError = Rsetpoint - Rinput;
  rollOutputDelta = rollK1 * rollError + rollK2 * rollErrorPrev1 + rollK3 * rollErrorPrev2;
  *Routput = constrain(*Routput - rollOutputDelta, -PIDbound, PIDbound);
}

//Calculate local gains
void calcK() {
  pitchK1 = pitchKp + pitchKi + pitchKd;
  pitchK2 = -pitchKp - (2 * pitchKd);
  pitchK3 = pitchKd;

  rollK1 = rollKp + rollKi + rollKd;
  rollK2 = -rollKp - (2 * rollKd);
  rollK3 = rollKd;
}

//Zero PID vars
void PIDzero() {
  pitchOutput = 0; pitchError = 0; pitchErrorPrev1 = 0; pitchErrorPrev2 = 0;
  rollOutput = 0; rollError = 0;  rollErrorPrev1 = 0;  rollErrorPrev2 = 0;
}
