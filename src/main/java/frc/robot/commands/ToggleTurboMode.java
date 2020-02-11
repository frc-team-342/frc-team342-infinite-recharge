/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Factory;
import frc.robot.subsystems.DriveSystem;

public class ToggleTurboMode extends CommandBase {
  private static DriveSystem driveSystem;
  /**
   * Creates a new ToggleTurboMode.
   */
  public ToggleTurboMode() {
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
    if(driveSystem.getTurbo())
      driveSystem.setTurbo(false);
    else{
      driveSystem.setTurbo(true);
      driveSystem.setSlow(false);
    }
    SmartDashboard.putBoolean("Is Turbo Mode", driveSystem.getTurbo());
    SmartDashboard.putBoolean("Is Slow Mode", driveSystem.getSlow());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
