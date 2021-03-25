// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;

import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeAndOutake;
import frc.robot.Factory;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;

public class DriveToPowerCell extends CommandBase {
  DriveSystem drive;
  IntakeAndOutake intake;
  private Joystick joy;

  private double X;
  private double Y;
  
  PhotonCamera camera;
  double angle; // degrees from center of robot
  double distance;

  /** Creates a new DriveToPowerCell. */
  public DriveToPowerCell() {
    camera = new PhotonCamera("HD_USB_Camera");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    joy = RobotContainer.getJoy();
    drive = Factory.getDrive();
    intake = Factory.getIntakeOutake();
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

    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      System.out.println("Has Target");
      PhotonTrackedTarget target = result.getTargets().get(0);
      
      // turn towards ball
      angle = target.getYaw();
      System.out.println("Yaw of Powercell " + angle + "\nRobot Angle " + drive.getGyro() + "\nRobot Angle - Yaw of Powercell " + (drive.getGyro() + angle));
      drive.driveWithTargeting(0, 0, angle); // autorotate takes field-relative degree, not robot
      
      // drive to ball
      //drive.Drive(0.0, 0.0, 0.0);

      // intake ball

    } else {
      // uhh spin?? uhhhh hm huh uhh
      System.out.println("Its not working, yo");
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
