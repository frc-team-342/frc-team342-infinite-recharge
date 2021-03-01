// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;

import frc.robot.subsystems.DriveSystem;
import frc.robot.Factory;

public class DriveToPowerCell extends CommandBase {
  PhotonCamera camera;
  DriveSystem drive;
  double angle;

  /** Creates a new DriveToPowerCell. */
  public DriveToPowerCell() {
    camera = new PhotonCamera("photon");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      angle = target.getYaw();
      // roboto, rotat e.
      // rotato FASTER roboto
      drive.autoRotate(angle); // please do not run this
    } else {
      // uhh spin?? uhhhh hm huh uhh
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
