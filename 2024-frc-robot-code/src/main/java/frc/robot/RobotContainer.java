// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.HangerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  public static XboxController xboxController;
  public static Joystick stickLeft, stickRight;

  DriveSubsystem driveSub;
  IntakeSubsystem intakeSub;
  HangerSubsystem hangerSub;
  AutonomousSubsystem autoSub;

  DriveCommand driveCmd;
  IntakeCommand intakeCmd;
  HangerCommand hangerCmd;
  AutonomousCommand autoCmd;

  ShuffleChooser chooser;

  public RobotContainer() {
    xboxController = new XboxController(Constants.xboxPort);

    stickLeft = new Joystick(Constants.stickPortL);
    stickRight = new Joystick(Constants.stickPortR);

    driveSub = new DriveSubsystem();
    intakeSub = new IntakeSubsystem();
    hangerSub = new HangerSubsystem();
    autoSub = new AutonomousSubsystem(driveSub,intakeSub,hangerSub);

    chooser = new ShuffleChooser();
    shuffleInfo();

    driveCmd = new DriveCommand(driveSub);
    intakeCmd = new IntakeCommand(intakeSub);
    hangerCmd = new HangerCommand(hangerSub);
    
    driveSub.setDefaultCommand(driveCmd);
    intakeSub.setDefaultCommand(intakeCmd);
    hangerSub.setDefaultCommand(hangerCmd);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    // return new AutonomousCommand(autoSub, false, "CUBE");
    return new AutonomousCommand(autoSub, chooser.getStartLocation().getSelected(), chooser.getStartItem().getSelected());
  }

  public void shuffleInfo() {
    SmartDashboard.putData(this.chooser.getStartItem());
    SmartDashboard.putData(this.chooser.getStartLocation());
  }
}
