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

public class DriveWithJoystick extends CommandBase {
  private final DriveSystem driveSystem;
  private final Joystick joy;
  private double X;
  private double Y;
  private double Z;
  /**
   * Creates a new DriveWithJoystick.
   */
  public DriveWithJoystick() {
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
    X = joy.getX();
    Y = joy.getY();
    Z = joy.getZ();

    if(Math.abs(X) < 0.2)
      X = 0.0;
    if(Math.abs(Y) < 0.2)
      Y = 0.0;
    if(Math.abs(Z) < 0.2)
      Z = 0.0;
    driveSystem.Drive(X, Y, Z);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSystem.Drive(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
