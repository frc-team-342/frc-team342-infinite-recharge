// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import frc.robot.Factory;


public class DriveCycleBtn extends CommandBase {
  /* Allows a single toggle button to move the robot between the power port and load station so it isn't done manually*/
  private static DriveSystem driveSystem;

  public DriveCycleBtn() {
    driveSystem = Factory.getDrive();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*new AutoMove(0.45).withTimeout(0.8);  //forward
    new AutoTarget().withTimeout(2.0);  //allign 
    new LaunchWithButton(); //shoot
    new AutoMove(-0.45).withTimeout(0.8); //reverse */
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
