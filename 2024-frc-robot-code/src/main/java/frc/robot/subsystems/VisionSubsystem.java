// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class VisionSubsystem extends SubsystemBase {
//   NetworkTable limelightTable;
//   double cameraAngle = 0; //degrees
//   double cameraHeight; 
//   double lowerPole = 1.927; //feet from floor to center of tape
//   double upperPole = 3.573; //feet from floor to center of tape


//   /** Creates a new VisionSubsystem. */
//   public VisionSubsystem() {
//     limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   public double getTx() {
//     return limelightTable.getEntry("tx").getDouble(0.0);
//   }
//   public double getTy() {
//     return limelight   Table.getEntry("ty").getDouble(0.0);
//   }

//   public double getDistance() {
//     return (lowerPole-cameraHeight) / Math.tan(Math.toRadians(cameraAngle + getTy()));
//     //return (upperPole-cameraHeight) / Math.tan(Math.toRadians(cameraAngle + getTy()));
//   }

  
// }
