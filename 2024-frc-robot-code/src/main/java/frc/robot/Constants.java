// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
//  public static class OperatorConstants {
//    public static final int kDriverControllerPort = 0;
//  }

  //XboxController port **Need to verify port id**
  public static final int xboxPort = 0;

  //Joystick ports **Need to verify port id**
  public static final int stickPortL = 1;
  public static final int stickPortR = 2;

  /* Drive */
  //Wheel ports **Need to verify port id**
  public static final int CANPortL1 = 1; 
  public static final int CANPortL2 = 3; 

  public static final int CANPortR1 = 2; 
  public static final int CANPortR2 = 4; 

    public static final int CANPortHang1 = 6; 


  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;
  
  /*Hanger */
  public static final int CANPortShoot1 = 8; 
  public static final int CANPortShoot2 = 10; 


}
