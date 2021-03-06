// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeAndOutake;

public class ShootWithDelay extends CommandBase {
  IntakeAndOutake subsystem;

  /** Creates a new ShootWithDelay. */
  public ShootWithDelay() {
    // Use addRequirements() here to declare subsystem dependencies.
    subsystem = frc.robot.Factory.getIntakeOutake();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.outtakeWithDelay();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.shooterStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
