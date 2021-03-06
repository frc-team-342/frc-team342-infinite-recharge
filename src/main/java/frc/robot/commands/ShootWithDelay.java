// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeAndOutake;
import edu.wpi.first.wpilibj.Timer;

public class ShootWithDelay extends CommandBase {
  IntakeAndOutake subsystem;
  Timer timer;

  /** Creates a new ShootWithDelay. */
  public ShootWithDelay() {
    // Use addRequirements() here to declare subsystem dependencies.
    subsystem = frc.robot.Factory.getIntakeOutake();
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < 2.0) {
      timer.stop();
      subsystem.outake();
      timer.reset();
      timer.start();
    }
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
