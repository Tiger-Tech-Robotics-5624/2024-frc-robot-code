// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShootSubsystem;

public class ShootCommand extends Command {
  ShootSubsystem shootSub;
  /** Creates a new IntakeCommand. */
  public ShootCommand(ShootSubsystem shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    shootSub = shoot;
    addRequirements(shootSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // intakeSub.pullPushIntake(RobotContainer.xboxController.getLeftTriggerAxis(),RobotContainer.xboxController.getRightTriggerAxis());
    // intakeSub.lower(-RobotConta0iner.xboxController.getLeftY(),false);
    shootSub.setSpeed(RobotContainer.xboxController.getXButton());
    shootSub.shoot(RobotContainer.xboxController.getLeftBumper());
    shootSub.reverse(RobotContainer.xboxController.getRightBumper());
    // shootSub.slowIn(RobotContainer.xboxController.getRightBumper());
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
