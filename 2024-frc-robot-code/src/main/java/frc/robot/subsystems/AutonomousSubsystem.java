// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class AutonomousSubsystem {
  DriveSubsystem driveSub;
  IntakeSubsystem intakeSub;
  VisionSubsystem visionSub;
  HangerSubsystem hangerSub;
  Timer timer; 

  
  boolean teleop = false;

  /** Creates a new AutonomousSubsystem. */
  public AutonomousSubsystem(DriveSubsystem drive, IntakeSubsystem intake,HangerSubsystem hanger) {
    driveSub = drive;
    intakeSub = intake;
   hangerSub = hanger;
    visionSub = new VisionSubsystem();
    timer = new Timer();
  }

  public void periodic(){
    
  }

  public void start(){
    timer.reset();
    teleop = false;
    timer.start();
    driveSub.resetGyro();
   hangerSub.zeroEncoder();
    intakeSub.zeroEncoder();
  }

  public void stop() {
    teleop = true;
    intakeSub.stop();
    intakeSub.stopLower();
   hangerSub.stop();
    driveSub.stop();
  }
  
  public void runAutonomous(String item, boolean balance) {

    //Testing
    // while(timer.get() < 1000) {

    // }
      
    //Autonomous
    while(timer.get() < 15 && !teleop) {
      if(timer.get() > 0 && timer.get() < 1){
       hangerSub.autoMove(-0.35);
      }
      else if(timer.get() > 6 && timer.get() < 7) {
       hangerSub.autoMove(0.35);
      }
      else { 
       hangerSub.stop();
      }


      //Starting configuration

      //Starting with cube
      if(item == "CUBE") {
        if(timer.get() > 0 && timer.get() < 2.6) {
          intakeSub.autoLower(1, false);
        }
        else if(timer.get() > 2.6 && timer.get() < 3.6){
          intakeSub.autoLower(0,true);
          intakeSub.shoot(true);
        }
        else {
          intakeSub.stop();
        }
        if(timer.get() > 4 && timer.get() < 6) {
          intakeSub.autoLower(-0.38,false);
        }
        else {
          intakeSub.stopLower();
        }
      } 

      //Starting with cone
      else {
        if(timer.get() > 0 && timer.get() < 3.5) {
          intakeSub.autoLower(1, false);
        }
        else if(timer.get() > 3.5 && timer.get() < 4){
          intakeSub.autoLower(0,true);
          intakeSub.shoot(true);
        }
        else {
          intakeSub.stop();
        }
        if(timer.get() > 4 && timer.get() < 6) {
          intakeSub.autoLower(-1,false);
        }
        else {
          intakeSub.stopLower();
        }
      }

      //Drive Autonomous

      //Straight Drive
      if(!balance){
        if(timer.get() > 6 && timer.get() < 10.5 && !balance) {
          driveSub.autonomousDrive(-0.2, 0);
        }
        else {
          driveSub.stop();
        }
      }

      //Auto-balance??
      else {
        //Moving backwards
        // if(timer.get()>6 && driveSub.getAverageEncoder() < 69) {
        //   driveSub.autonomousDrive(-0.8, 0);
        // }


        
        //Turning and moving fowards
        if(timer.get()>6 && timer.get()<6.2) {
          driveSub.autonomousDrive(-0.2, 0);
        }

        else if(timer.get()>6.2 && timer.get() < 8){
          driveSub.autonomousDrive(0, 180);
          driveSub.zeroEncoder();
        }

        else if(timer.get()>8 && driveSub.getAverageEncoder()/Constants.kEncoder2Feet < 6) {
          driveSub.autonomousDrive(0.8, 180);
        }



        //Attempt autobalance after it is on charge pad
        else if(driveSub.getAverageEncoder()/Constants.kEncoder2Feet > 6) {
          driveSub.autoBalance();
        }
      }

    }

    
  }
}
