//local PID vars
double pitchError, pitchErrorPrev = 0;
double rollError, rollErrorPrev = 0;

//PID calculation
void PIDcalc(double Pinput, double Psetpoint, double *Poutput, double Rinput, double Rsetpoint, double *Routput) {
  pitchError = Psetpoint + Pinput;
  pitchErrorSum += (pitchError * sampleTime);
  *Poutput = pitchKp * pitchError + pitchKi * pitchErrorSum + pitchKd * (pitchError - pitchErrorPrev) / sampleTime;
  pitchErrorPrev = pitchError;

  rollError = Rsetpoint + Rinput;
  rollErrorSum += (rollError * sampleTime);
  *Routput = rollKp * rollError + rollKi * rollErrorSum + rollKd * (rollError - rollErrorPrev) / sampleTime;
  rollErrorPrev = rollError;
}
