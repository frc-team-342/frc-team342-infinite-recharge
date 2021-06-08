// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrajectoryAuto extends SequentialCommandGroup {
  DriveSystem drive;
  
  /** the competition autonomous using trajectories */
  public TrajectoryAuto() {
    /** 
     * the angle that the robot starts the autonomous at. used to reset to the original angle so that we can continue to run trajectories afterwards.
     */
    double startAngle = drive.getGyro();
    
    addCommands(
      new RotateToAngle(50),
      new RotateToAngle(startAngle - drive.getGyro()), // rotate back to the angle that it started at
      new PrintCommand("" + drive.getGyro())
    );
  }
}
