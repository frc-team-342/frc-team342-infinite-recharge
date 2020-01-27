/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import frc.robot.Factory; 

public class RotateToAngle extends CommandBase {
  /**
   * Rotates the robot to a given angle
   */
  
  private final DriveSystem driveSystem; 

  private double angle;
  private double gyro_angle; 

  private boolean TurnRight;

  private static final double RotateSpeed = 1.0;
  private static final double RotateSlowSpeed = 0.5; 
  private static final double margin = 5;
	private static final double slowmargin=45;
	private static final double SPEED = 0.7;

  public RotateToAngle(int angle) {
    driveSystem = Factory.getDrive(); 

    this.angle = angle; 
  }

  @Override
  public void initialize() {
    driveSystem.resetGyro(); 
  }

  @Override
  public void execute() {

    double CurrentDriveSpeed;
		boolean slowdown = (gyro_angle) <= (angle) + slowmargin && (gyro_angle) >= (angle) - slowmargin;
    
    gyro_angle = driveSystem.getGyro(false); 

    double diff = angle - gyro_angle; 
    if(diff<0){
      diff = diff + 360;
    } else if (diff > 180){
      TurnRight = false;
    } else {
      TurnRight = true; 
    }
    
    if(slowdown){
      CurrentDriveSpeed = RotateSlowSpeed;
    } else {
      CurrentDriveSpeed = RotateSpeed;
    }

    if(TurnRight){
      driveSystem.Drive(CurrentDriveSpeed * SPEED, CurrentDriveSpeed * SPEED, CurrentDriveSpeed); 
    } else {
      driveSystem.Drive(-CurrentDriveSpeed * SPEED, -CurrentDriveSpeed * SPEED, -CurrentDriveSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSystem.stopDrive(); 
  }

  @Override
  public boolean isFinished() {

		boolean isFinished = (gyro_angle) <= (angle) + margin && (gyro_angle) >= (angle) - margin;
		
		if (isFinished) {
			return true;
		} else {
			return false;
    }
  }
}
