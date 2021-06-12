// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.Factory;
import frc.robot.subsystems.DriveSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrajectoryAuto extends SequentialCommandGroup {
  DriveSystem drive = Factory.getDrive();
  
  /** the competition autonomous using trajectories */
  public TrajectoryAuto(TrajectoryConfig config, Trajectory trajectory) {
     
    // the angle that the robot starts the autonomous at. used to reset to the original angle so that we can continue to run trajectories afterwards.
    double startAngle = drive.getGyro();

    // ramsete command to follow trajectory
    var ramsete = new RamseteCommand(
      trajectory, // passed as an argument from robotcontainer
      drive::getPose2d, // used to get current position of robot
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(
        Constants.ksVolts, 
        Constants.kvVoltsSecondsPerMeter, 
        Constants.kaVoltsSecondsSquaredPerMeter
      ),
      Constants.kDifferentialKinematics, 
      drive::getDifferentialWheelSpeeds, // get current wheel speeds of robot
      new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel), 
      new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
      drive::differentialDriveVolts, // function used to drive the robot with volts as input
      drive
    );
    
    addCommands(
      ramsete, // move forwards using trajectory defined in robotcontainer
      new RotateToAngle(-135.0), // rotate so that the limelight can see the target
      new AutoTarget().withTimeout(1.0), 
      new LaunchWithButton(),
      new RotateToAngle(startAngle) // rotate back to the angle that the robot started autonomous at
    );
  }
}
