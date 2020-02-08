/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Factory;
import frc.robot.subsystems.DriveSystem;

public class AutoMove extends CommandBase {
  private static DriveSystem driveSystem;
  private Command autoMove;
  /**
   * Creates a new AutoMove.
   */
  public AutoMove() {
    driveSystem = Factory.getDrive();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    autoMove = new StartEndCommand(
      () -> driveSystem.autoDrive(0.0, 0.0, 0.0),
      
      () -> driveSystem.autoDrive(0.0, 0.0, 0.0),
      driveSystem).withTimeout(0.0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
