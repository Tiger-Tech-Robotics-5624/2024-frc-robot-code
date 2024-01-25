// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutonomousSubsystem;

public class AutonomousCommand extends Command {

  AutonomousSubsystem autoSub;
  boolean location;
  String item;
  /** Creates a new AutonomousCommand. */
  public AutonomousCommand(AutonomousSubsystem auto, boolean location, String item) {
    // Use addRequirements() here to declare subsystem dependencies.
    autoSub = auto;
    this.location = location;
    this.item = item;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoSub.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    autoSub.runAutonomous(this.item,this.location);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoSub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}