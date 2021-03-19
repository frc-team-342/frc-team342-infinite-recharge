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

public class DriveToPowerCell extends CommandBase {
  DriveSystem drive;
  IntakeAndOutake intake;
  
  PhotonCamera camera;
  double angle; // degrees from center of robot
  double distance;

  /** Creates a new DriveToPowerCell. */
  public DriveToPowerCell() {
    camera = new PhotonCamera("camera1");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive = Factory.getDrive();
    intake = Factory.getIntakeOutake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      
      // turn towards ball
      angle = target.getYaw();
      System.out.println("Yaw of Powercell " + angle + "\nRobot Angle " + drive.getGyro() + "\nRobot Angle - Yaw of Powercell " + (drive.getGyro() - angle));
      //drive.autoRotate(drive.getGyro() - angle); // autorotate takes field-relative degree, not robot
      
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
