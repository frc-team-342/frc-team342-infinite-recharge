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

public class RotateToAngle extends CommandBase {
  private final DriveSystem driveSystem;
  private double gyro;
  private double angle;
  /**
   * Creates a new RotateToAngle.
   */
  public RotateToAngle(double Angle) {
    driveSystem = Factory.getDrive();
    angle = Angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSystem.zeroGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyro = driveSystem.getGyro();
    if(angle > 180){
      while(gyro != angle){
        driveSystem.Drive(0.0, 0.0, -0.3);
      }
    }
    else if(angle <= 180){
      while(gyro != angle){
        driveSystem.Drive(0.0, 0.0, 0.3);
      }
    }
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
