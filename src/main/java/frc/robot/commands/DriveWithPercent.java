/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Factory;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSystem;

public class DriveWithPercent extends CommandBase {
  private final DriveSystem driveSystem;
  private final Joystick joy;
  private double Y;
  /**
   * Creates a new DriveWithPercent.
   */
  public DriveWithPercent() {
    driveSystem = Factory.getDrive();
    joy = RobotContainer.getJoy();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Y = joy.getY();
    driveSystem.PercentOut(Y);
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
