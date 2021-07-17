// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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

  double startAngle;

  PhotonCamera camera;
  
  /** the competition autonomous using trajectories */
  public TrajectoryAuto(TrajectoryConfig config, Trajectory trajectory) {
     
    // the angle that the robot starts the autonomous at. used to reset to the original angle so that we can continue to run trajectories afterwards.
    startAngle = drive.getGyro();

    // camera info is input from photonvisioon
    camera = new PhotonCamera("HD_USB_Camera");

    // ramsete command to follow trajectory
    var ramseteStart = new RamseteCommand(
      startTrajectory, // passed as an argument from robotcontainer
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

    //This was scraped on July 17th 2021
    /*var ramseteTrench = new RamseteCommand(
      trenchTrajectory, // passed as an argument from robotcontainer
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
    );*/    

    var ramseteEnd = new RamseteCommand(
      endTrajectory, // passed as an argument from robotcontainer
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
      /* turn off limelight and reset starting angle */
      new InstantCommand(() -> {
        startAngle = drive.getGyro(); // makes sure the start angle resets every run
        Factory.getLimelight().visionOff(); // turns the limelight off
      }), // turn off limelight

      /* rotate to be able to see target */
      new RotateToAngle(drive.getGyro() - 30.0), 
      new PrintCommand("First Rotate Successful"),

      /* target and shoot while intaking */
      new AutoTarget().withTimeout(1.0),
      new ParallelRaceGroup( // runs both commands at the same time until one finishes
        new LaunchWithButton().withTimeout(3.2), // shoot for that amount of time
        new IntakeWithButton() // run the intake while shooting in case power cell gets stuck
      ),

      /* rotate to face the trench */
      new RotateToAngle(drive.getGyro() - 150),

      /* line up by turning towards a power cell */
      new ConditionalCommand( // runs one command or another based on the boolean expression in the third argument
        // runs if third argument is true
        new RotateToAngle(drive.getGyro() - camera.getLatestResult().getTargets().get(0).getYaw()), // rotates to where the robot sees the powercell
        // runs if third argument is false
        new InstantCommand(), // does nothing, 
        camera.getLatestResult()::hasTargets // checks if the camera is seeing powercells
      ),      

      /* drive forwards and intake */
      new ParallelRaceGroup( // runs both commands at the same time until one finishes
        ramseteStart, // drive will finish first because intake does not have an end condition
        new IntakeWithButton() // run the intake while driving
      ),      

      /* rotate back to the angle that it started at */
      new RotateToAngle(startAngle), // so that the robot can continue and know its angle and position after
      new PrintCommand("Second Rotate Successful"),

      /* drives to pick up the powercells in the trench run */
      new ParallelRaceGroup( // runs both commands at the same time until one finishes
        ramseteEnd, // drive will finish first because intake does not have an end condition
        new IntakeWithButton() // run the intake while driving
      ), 

      /* rotate towards powerport */
      new RotateToAngle(drive.getGyro() - 30),

      /* target and shoot while intaking */
      new AutoTarget().withTimeout(1.0),
      new ParallelRaceGroup( // runs both commands at the same time until one finishes
        new LaunchWithButton().withTimeout(3.2), // shoot for that amount of time
        new IntakeWithButton() // run the intake while shooting in case power cell gets stuck
      )      
    );
  }
}
