/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Factory;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.Joystick;

public class AutoTarget extends CommandBase {
  private final LimelightSubsystem lime;
  private final DriveSystem driveSystem;
  private double error = 0.5;
  private boolean isDone = false;

  /**
   * Creates a new RotateToAngle.
   */
  public AutoTarget() {
    driveSystem = Factory.getDrive();

    lime = Factory.getLimelight(); 


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lime.visionOn();
    driveSystem.errorAccumReset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSystem.driveWithTargeting(0.0, 0.0, lime.getXOffsetAngle());

    //if(Math.abs(lime.getXOffsetAngle()) - error < 0.6)
    //  isDone = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;

  }
}
