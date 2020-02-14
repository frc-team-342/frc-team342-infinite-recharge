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
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.LimelightSubsystem;

public class RotateToAngle extends CommandBase {
  private final LimelightSubsystem lime;
  private final DriveSystem driveSystem;
  private double error = 0.5;
  private boolean isDone = false;
  /**
   * Creates a new RotateToAngle.
   */
  public RotateToAngle() {
    driveSystem = Factory.getDrive();
    lime = Factory.getLime();
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSystem.errorAccumReset();
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSystem.autoRotate(lime.getXOffsetAngle());
    if(Math.abs(lime.getXOffsetAngle())<error && lime.getValidTarget()){
      driveSystem.stopDrive();
      isDone = true;
    }
}
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSystem.stopDrive();
    System.out.println("yuh");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isDone)
      return true;
    else
      return false;
    
  }
}
