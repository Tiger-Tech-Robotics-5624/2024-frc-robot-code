// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
//!!import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  IntakeSubsystem intakeSub;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSub = intake;
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSub.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.pullPushIntake(RobotContainer.xboxController.getLeftTriggerAxis(),RobotContainer.xboxController.getRightTriggerAxis());
    intakeSub.lower(-RobotContainer.xboxController.getLeftY(),false);
    intakeSub.place(RobotContainer.xboxController.getAButton());
    intakeSub.shoot(RobotContainer.xboxController.getBButton());
    intakeSub.slowIn(RobotContainer.xboxController.getRightBumper());
    // intakeSub.pidTest();
    // intakeSub.pidTestStart(RobotContainer.xboxController.getXButton(), RobotContainer.xboxController.getYButton());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
