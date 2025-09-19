void curveTurn(bool dir){
  double R_center = 25.0;
  double center_distance = R_center * 3.14159 * 0.5;
  update();
  reset();
  double t0 = micros();
  double delta_T = turnTime;
  double delta_T_us = delta_T * 1e3;
  double left_pwm = str_min;
  double right_pwm = str_min;
  double velocity_setpoint = 0;
  double elapsed_time;
  double ratio_outer = 1.5;
  double target_ratio = 0.6666667;
  double eps = 1e-6;

  while (true)
  {
    elapsed_time = micros() - t0;
    update();
    double avgDist = 0.5 * (dL() + dR());
    if (avgDist >= center_distance)
    {
      break;
    }

    if (elapsed_time <= delta_T_us / 4)
    {
      velocity_setpoint = (16.0 * center_distance) / (3.0 * delta_T * delta_T) * (elapsed_time / 1e6);
    }
    else if (elapsed_time <= 3 * delta_T_us / 4)
    {
      velocity_setpoint = (4.0 * center_distance) / (3.0 * delta_T);
    }
    else
    {
      double t_dec = elapsed_time - 3 * delta_T_us / 4;
      velocity_setpoint = (16.0 * center_distance) / (3.0 * delta_T * delta_T) * ((delta_T / 4) - t_dec / 1e6);
    }

    if(dir){
      velocity_error_L = velocity_setpoint - vL();
      velocity_error_R = ratio_outer * velocity_setpoint - vR();
    }else{
      velocity_error_L = ratio_outer * velocity_setpoint - vL();
      velocity_error_R = velocity_setpoint - vR();
    }

    left_pwm += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R;

    left_pwm = constrain(left_pwm, str_min, 400);
    right_pwm = constrain(right_pwm, str_min, 400);

    if(dir){
      double denom = fabs(vR()) < eps ? eps : vR();
      right_pwm -= swerve_Kpt * ((vL()/denom) - target_ratio);
    }else{
      double denom = fabs(vL()) < eps ? eps : vL();
      left_pwm -= swerve_Kpt * ((vR()/denom) - target_ratio);
    }

    motors.setSpeeds(left_pwm, right_pwm);
  }

  motors.setSpeeds(0, 0);
  delay(10 + delayer_amt);
  reset();
}