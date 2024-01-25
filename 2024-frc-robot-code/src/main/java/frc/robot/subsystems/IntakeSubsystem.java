// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  //Initilize Motors Here
  private TalonSRX talon;
  private CANSparkMax spark;
  private RelativeEncoder s_encoder;

  ShuffleboardTab tab;
  GenericEntry kPEntry;
  GenericEntry kIEntry;
  GenericEntry kDEntry;
  GenericEntry PIDSpeedValue;
  GenericEntry setPoint;

  private PIDController PID;
  double kp = 0; //Proportional
  double ki = 0; //Integral
  double kd = 0; //Derivative

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    //Define Motors Here
    talon = new TalonSRX(Constants.TalonPort1); //INTAKE
    spark = new CANSparkMax(Constants.SparkPort1, MotorType.kBrushless); //ELBOW
    s_encoder = spark.getEncoder();

    
    tab = Shuffleboard.getTab("PID Testing");
    kPEntry = tab.addPersistent("kP", 0).getEntry();
    kIEntry = tab.addPersistent("kI", 0).getEntry();
    kDEntry = tab.addPersistent("kD", 0).getEntry();
    PIDSpeedValue = tab.addPersistent("PID Speed", 0).getEntry();
    setPoint = tab.addPersistent("Set Point", 0).getEntry();
    
    PID = new PIDController(kp, ki, kd);
    PID.setTolerance(1);
    //Range of Integral values
    PID.setIntegratorRange(-0.5, 0.5);
  }

  
  public void pidTestStart(boolean xButton,boolean yButton) {
    if(xButton) {
      PID.setSetpoint(3.9);
    }
    else if(yButton) {
      zeroEncoder();
      PID.setSetpoint(0);
      SmartDashboard.putNumber("Setpoint", 0);
    }
  }

  public void pidTest() {
    SmartDashboard.putNumber("Intake Encoder Value", Math.abs(s_encoder.getPosition()));

    //**Tuning PID**
    //Slowly increase P by small increments and test the response
    //Eventually, get your P so that it's smooth, and if it oscillates a little, that's okay
    //Next, increase D slowly until the oscillations go away
    //Finally, increase I until it is able to home into the desired target position

    //Not sure if this is bad but if so, then just use the commented out version insead and reboot code :(
    PID.setPID(kPEntry.getDouble(0),kIEntry.getDouble(0),kDEntry.getDouble(0));
    
    double PIDSpeed = MathUtil.clamp(PID.calculate(s_encoder.getPosition()/Constants.kEncoder2Feet),-0.75,0.75); //Using the 10 ft test for the encoder value to feet conversion
    PIDSpeedValue.setDouble(PIDSpeed);
    // PID.setSetpoint(setPoint.getDouble(0));
    SmartDashboard.putNumber("PIDSpeed", PIDSpeed);

    //Moves foward when xButton pressed, backwards when yButton pressed.
    spark.set(PIDSpeed);
  }

  public void pullPushIntake(double leftTAxis,double rightTAxis) {
    if(leftTAxis > 0.03 && rightTAxis <= 0.03) {
      talon.set(TalonSRXControlMode.PercentOutput,-leftTAxis * .50); //Push
    }
    else if(rightTAxis > 0.03 && leftTAxis <= 0.03){
      talon.set(TalonSRXControlMode.PercentOutput,rightTAxis * .50); //Pull
    }
    else {
      stop();
    }
  }

  public void place(boolean buttonA) {
    if(buttonA) {
      talon.set(TalonSRXControlMode.PercentOutput, -0.15);
    }
  }

  public void shoot (boolean buttonB) {
    if(buttonB) {
      talon.set(TalonSRXControlMode.PercentOutput, -0.35);
    }
  }

  public void slowIn(boolean rBumper) {
    if(rBumper) {
      talon.set(TalonSRXControlMode.PercentOutput, 0.15);
    }
  }

  public void stop() {
    talon.set(TalonSRXControlMode.PercentOutput,0);
  }

  public void lower(double yAxis,boolean rBumper) {
    // SmartDashboard.putNumber("Lower yAxis", yAxis);
    // SmartDashboard.putNumber("Elbow Encoder Value", s_encoder.getPosition());
    // SmartDashboard.putNumber("Elbow Velocity", s_encoder.getVelocity());
    if(yAxis > 0.1){
      spark.setInverted(false);
      spark.set(0.45 * yAxis);
    }
    else if(yAxis < -0.1) {
      spark.setInverted(true);
      spark.set(-0.6 * yAxis); 
    }
    else if(rBumper) {
      spark.setInverted(false);
      spark.set(-0.25);
    }
    
    else {
      stopLower();
    }
  }

  public void autoLower(double yAxis,boolean rBumper) {
    if(yAxis > 0.1){
      spark.setInverted(false);
      spark.set(yAxis);
    }
    else if(yAxis < -0.1) {
      spark.setInverted(true);
      spark.set(-yAxis); 
    }
    else if(rBumper) {
      spark.setInverted(false);
      spark.set(-0.25);
    }
    else {
      stopLower();
    }
  }
  public double getEncoderPos() {
    return s_encoder.getPosition();
  }

  public void zeroEncoder() {
    s_encoder.setPosition(0);
  }

  public void stopLower() {
    // spark.set(0);
    spark.setVoltage(0);
    // spark.disable();
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void resetEncoder() {
    s_encoder.setPosition(0);
  }

}
  