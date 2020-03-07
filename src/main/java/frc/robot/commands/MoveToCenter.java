/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Factory;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.LimelightSubsystem;

public class MoveToCenter extends CommandBase {
  /**
   * Creates a new MoveToCenter.
   */
  private LimelightSubsystem lime;
  private DriveSystem drive;
  private double X;
  public MoveToCenter() {
    // Use addRequirements() here to declare subsystem dependencies.
    lime = Factory.getLimelight();
    drive = Factory.getDrive();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    X = lime.getXOffsetAngle();
    System.out.println(X);
    System.out.println(lime.getXOffsetAngle());
    if (X < -1) { 
      drive.Drive(-0.2, 0.0, 0.0);
      System.out.println("Bruh Moment");
    }
    else if (X > 1) {
      drive.Drive(0.2, 0.0, 0.0);
      System.out.println("Bruh Moment");
    }
    else if (X > -1 && X < 1) {
      drive.Drive(0.0, 0.0, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if (X > -1 && X < 1) {
      return false;
    }
    else {
      return true;
    }*/
    return true;
  }
}
