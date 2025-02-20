// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  public static XboxController xboxController;
  public static Joystick stickLeft, stickRight;

  static DriveSubsystem driveSub;
  static ShootSubsystem shootSub;

  DriveCommand driveCmd;
  ShootCommand shootCmd;
  ShuffleChooser chooser;

  public RobotContainer() {
    xboxController = new XboxController(Constants.xboxPort);

    stickLeft = new Joystick(Constants.stickPortL);
    stickRight = new Joystick(Constants.stickPortR);

    driveSub = new DriveSubsystem();
    shootSub = new ShootSubsystem();
    

    chooser = new ShuffleChooser();
    shuffleInfo();

    driveCmd = new DriveCommand(driveSub);
    shootCmd = new ShootCommand(shootSub);
    
    driveSub.setDefaultCommand(driveCmd);
    shootSub.setDefaultCommand(shootCmd);
  }

  private void configureBindings() {}


  public void shuffleInfo() {
  }
}
