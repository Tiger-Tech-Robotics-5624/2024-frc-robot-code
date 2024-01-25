// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkMax neo;
  private RelativeEncoder e_encoder;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    neo = new CANSparkMax(Constants.CANPortElevator, MotorType.kBrushless);
    e_encoder = neo.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void verticalMove(double yAxis) {
    // SmartDashboard.putNumber("Elevator Encoder (Manual)", e_encoder.getPosition());
    if(yAxis > 0.25 || yAxis < -0.25){
      neo.set((0.35 * yAxis));
    }
    else {
      stop();
    }
  }

  public void autoMove(double value) {
    // SmartDashboard.putNumber("Elevator Encoder (Auto)", e_encoder.getPosition());
    neo.set(value);
  }

  public void stop() {
    neo.set(-0.075); 
  }

  public void zeroEncoder() {
    e_encoder.setPosition(0);
  }
}
