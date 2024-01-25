// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  private CANSparkMax motorR1; //ID 2
  private CANSparkMax motorR2; //ID 4

  private CANSparkMax motorL1; //ID 1
  private CANSparkMax motorL2; //ID 3

  private RelativeEncoder encoderR1;
  private RelativeEncoder encoderR2;

  private RelativeEncoder encoderL1;
  private RelativeEncoder encoderL2;

  private MotorControllerGroup rightGroup;
  private MotorControllerGroup leftGroup;

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

    rightGroup = new MotorControllerGroup(motorR1, motorR2);
    leftGroup = new MotorControllerGroup(motorL1, motorL2);

    gyro = new AHRS(SPI.Port.kMXP);
    PID = new PIDController(kp, ki, kd);
    PID.setTolerance(1);
    //Range of Integral values
    PID.setIntegratorRange(-0.5, 0.5);

    encoderR1 = motorR1.getEncoder();
    encoderR2 = motorR2.getEncoder();
    encoderL1 = motorL1.getEncoder();
    encoderL2 = motorL2.getEncoder();
    
    // tab = Shuffleboard.getTab("PID Testing");
    // kPEntry = tab.addPersistent("kP", 0).getEntry();
    // kIEntry = tab.addPersistent("kI", 0).getEntry();
    // kDEntry = tab.addPersistent("kD", 0).getEntry();
    // PIDSpeedValue = tab.addPersistent("PID Speed", 0).getEntry();
  }

  public void pidTest() {
    // SmartDashboard.putNumber("Average Encoder Value", getAverageEncoder()/Constants.kEncoder2Feet);

    //Attempts to drive 6 ft with tolerance of 1 ft

    //**Tuning PID**
    //Slowly increase P by small increments and test the response
    //Eventually, get your P so that it's smooth, and if it oscillates a little, that's okay
    //Next, increase D slowly until the oscillations go away
    //Finally, increase I until it is able to home into the desired target position

    //Not sure if this is bad but if so, then just use the commented out version insead and reboot code :(
    // PID.setPID(kPEntry.getDouble(0),kIEntry.getDouble(0),kDEntry.getDouble(0));
    //PID.setPID(kp,kI,kD);
    
    // double PIDSpeed = MathUtil.clamp(PID.calculate(getAverageEncoder()/Constants.kEncoder2Feet),-0.75,0.75); //Using the 10 ft test for the encoder value to feet conversion
    // PIDSpeedValue.setDouble(PIDSpeed);
    //Moves foward when xButton pressed, backwards when yButton pressed.
    // rightGroup.set(-PIDSpeed);
    // leftGroup.set(PIDSpeed);
  }

  // public void pidTestStart(boolean xButton,boolean yButton) {
  //   if(xButton) {
  //     PID.setSetpoint(6);
  //   }
  //   else if(yButton) {
  //     zeroEncoder();
  //     PID.setSetpoint(0);
  //   }
  // }

  public void drive(double leftY, double rightY, double analogRead) 
  {
    if(rightY > 0.05 || rightY < -0.05 || leftY > 0.05 || leftY < -0.05){
      rightGroup.set(0.85 * (rightY * (0.50 - (0.25 * analogRead))) );
      leftGroup.set(0.85 * (-leftY * (0.50 - (0.25 * analogRead))) ); 
    }
    else{
      stop();
    }
  }

  // public void brakeMode(boolean b1){
  //   if(b1){
  //     motorR1.setIdleMode(IdleMode.kBrake);
  //     motorR2.setIdleMode(IdleMode.kBrake);
  //     motorL1.setIdleMode(IdleMode.kBrake);
  //     motorL2.setIdleMode(IdleMode.kBrake);
    
  //    motorR1.burnFlash();
  //    motorR2.burnFlash();
  //    motorL1.burnFlash();
  //    motorL2.burnFlash();
  //   }
  // }
  // public void coastMode(boolean b2){
  //    if(b2) {
  //   motorR1.setIdleMode(IdleMode.kBrake);
  //   motorR2.setIdleMode(IdleMode.kBrake);
  //   motorL1.setIdleMode(IdleMode.kBrake);
  //   motorL2.setIdleMode(IdleMode.kBrake);
    
  //   motorR1.burnFlash();
  //   motorR2.burnFlash();
  //   motorL1.burnFlash();
  //   motorL2.burnFlash();
  //  }
  // }

  public void autonomousDrive(double speed, double target) {
    error = target - gyro.getAngle();
    SmartDashboard.putNumber("Gryo Angle", gyro.getAngle());
    SmartDashboard.putNumber("Error", error);
    // value = drivePID.calculate(error);
    value = error * 0.1;
    if(value > 0.25) {
      leftGroup.set(0.05);
      rightGroup.set(0.05);
    }

    else if(value < -0.25) {
      rightGroup.set(-0.05);
      leftGroup.set(-0.05);
    }
    else {
      rightGroup.set(speed);
      leftGroup.set(-speed);
    }
  }

  //AutoBalance
  public double getAverageEncoder() {
    return (Math.abs(encoderR1.getPosition()) + Math.abs(encoderR2.getPosition()) + Math.abs(encoderL1.getPosition()) + Math.abs(encoderL2.getPosition())) / 4;
  }

  public void autoBalance(){
    if(gyro.getRoll()+4 > 10){
      rightGroup.set(-0.1);
      leftGroup.set(0.1);
    }
    else if(gyro.getRoll()+4 < -10){
      rightGroup.set(0.1);
      leftGroup.set(-0.1);
    }
    else if(getAverageEncoder()/Constants.kEncoder2Feet > 5.5 && getAverageEncoder()/Constants.kEncoder2Feet < 6.5 && gyro.getRoll() < 10 && gyro.getRoll() > -10) {
      stop();
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
    rightGroup.set(0);
    leftGroup.set(0);
  }

  public void zeroEncoder() {
    encoderR1.setPosition(0);
    encoderR2.setPosition(0);
    encoderL1.setPosition(0);
    encoderL2.setPosition(0);
  }
}