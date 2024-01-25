// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class PID {
  double kp;
  double ki;
  double kd;


  double i = 0;
  /** Creates a new PID. */
  public PID(double kp, double ki, double kd) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
  }

  public double calculate(double error) {
    i += error * 0.02f;
    double value = kp * error + ki * i;
    return value;
  }
}
