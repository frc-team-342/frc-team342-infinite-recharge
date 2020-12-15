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
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.Joystick;

public class DriveWithTargeting extends CommandBase {
  private final LimelightSubsystem lime;
  private final DriveSystem driveSystem;
  private final Joystick joy;
  private double error = 0.5;
  private double leftCorrection = 0.1;
  private boolean isDone = false;
  private double X;
  private double Y;

  /**
   * Creates a new RotateToAngle.
   */
  public DriveWithTargeting() {
    joy = RobotContainer.getJoy();
    driveSystem = Factory.getDrive();

    lime = Factory.getLimelight(); 


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lime.visionOn();
    driveSystem.errorAccumReset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    X = joy.getX();
    Y = joy.getY();

    if (Math.abs(X) < 0.15) {
      X = 0.0;
      SmartDashboard.putString("X Deadzone", "X is in deadzone!");
    } else
      SmartDashboard.putString("X Deadzone", "X is not in deadzone!");

    if (Math.abs(Y) < 0.15) {
      Y = 0.0;
      SmartDashboard.putString("Y Deadzone", "Y is in deadzone!");
    } else
      SmartDashboard.putString("Y Deadzone", "Y is not in deadzone!");

    if(lime.isLeft()){
      double offset = lime.getXOffsetAngle() + (lime.getLeftSkew() * leftCorrection);
      driveSystem.driveWithTargeting(X, -Y, offset);
    } else{
      driveSystem.driveWithTargeting(X, -Y, lime.getXOffsetAngle());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSystem.stopDrive();
    lime.switchCamMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isDone)
      return true;
    else
      return false;

  }
}
