// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
//Idle changed recently, idr what it is now but its in frc patches
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PID;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  public CANSparkMax motorR1; //ID 2
  private CANSparkMax motorR2; //ID 4

  private CANSparkMax motorL1; //ID 1
  private CANSparkMax motorL2; //ID 3


  //PID stuff
  private AHRS gyro;
  private PIDController PID;
  double kp = 0; //Proportional
  double ki = 0; //Integral
  double kd = 0; //Derivative
  
  double error;
  double value;
  double autoRightSpeed;
  double autoLeftSpeed;
  double autoSpeed;

  ShuffleboardTab tab;
  GenericEntry kPEntry;
  GenericEntry kIEntry;
  GenericEntry kDEntry;
  GenericEntry PIDSpeedValue;

  public DriveSubsystem() {

    motorR1 = new CANSparkMax(Constants.CANPortR1, MotorType.kBrushless);
    motorR2 = new CANSparkMax(Constants.CANPortR2, MotorType.kBrushless);

    motorL1 = new CANSparkMax(Constants.CANPortL1, MotorType.kBrushless);
    motorL2 = new CANSparkMax(Constants.CANPortL2, MotorType.kBrushless);


    gyro = new AHRS(SPI.Port.kMXP);
    PID = new PIDController(kp, ki, kd);
    PID.setTolerance(1);
    //Range of Integral values
    PID.setIntegratorRange(-0.5, 0.5);

   
    
    // tab = Shuffleboard.getTab("PID Testing");
    // kPEntry = tab.addPersistent("kP", 0).getEntry();
    // kIEntry = tab.addPersistent("kI", 0).getEntry();
    // kDEntry = tab.addPersistent("kD", 0).getEntry();
    // PIDSpeedValue = tab.addPersistent("PID Speed", 0).getEntry();
  }

    

  public void drive(double leftY, double rightY, double analogRead) 
  {
    if(rightY > 0.05 || rightY < -0.05 || leftY > 0.05 || leftY < -0.05){
      motorR1.set(0.865*(rightY * (0.50 - (0.25 * analogRead))) );
      motorR2.set(0.865*  (rightY * (0.50 - (0.25 * analogRead))) );
      motorL1.set(1*  (-leftY * (0.50 - (0.25 * analogRead))) ); 
      motorL2.set(1*  (-leftY * (0.50 - (0.25 * analogRead))) );
    }
    else{
      stop();
    }
  }

  public void driveAuto()
  {
      
        motorR1.set(-0.092);
        motorR2.set(-0.092);
        motorL1.set(0.1); 
        motorL2.set(0.1);
      
  }

  public void brakeMode(boolean b1){
    if(b1){
      motorR1.setIdleMode(IdleMode.kBrake);
      motorR2.setIdleMode(IdleMode.kBrake);
      motorL1.setIdleMode(IdleMode.kBrake);
      motorL2.setIdleMode(IdleMode.kBrake);
    
     motorR1.burnFlash();
     motorR2.burnFlash();
     motorL1.burnFlash();
     motorL2.burnFlash();
    }
  }
  public void coastMode(boolean b2){
     if(b2) {
    motorR1.setIdleMode(IdleMode.kBrake);
    motorR2.setIdleMode(IdleMode.kBrake);
    motorL1.setIdleMode(IdleMode.kBrake);
    motorL2.setIdleMode(IdleMode.kBrake);
    
    motorR1.burnFlash();
    motorR2.burnFlash();
    motorL1.burnFlash();
    motorL2.burnFlash();
   }
  }

  public void autonomousDrive(double speed, double target) {
    error = target - gyro.getAngle();
    SmartDashboard.putNumber("Gryo Angle", gyro.getAngle());
    SmartDashboard.putNumber("Error", error);
    // value = drivePID.calculate(error);
    value = error * 0.1;
    if(value > 0.25) {
      motorL1.set(0.05);
      motorL2.set(0.05);
      motorR1.set(0.05);
      motorR2.set(0.05);
    }

    else if(value < -0.25) {
      motorR1.set(-0.05);
      motorR2.set(-0.05);
      motorL1.set(-0.05);
      motorL2.set(-0.05);
    }
    else {
      motorR1.set(speed);
      motorR2.set(speed);
      motorL1.set(-speed);
      motorL2.set(-speed);
    }
  }





  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void resetGyro() {
    gyro.reset();
  }

  public void stop(){
    motorR1.set(0);
    motorR2.set(0);
    motorL1.set(0);
    motorL2.set(0);
  }


}