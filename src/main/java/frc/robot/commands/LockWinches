/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Factory;
import frc.robot.subsystems.ClimbSubsystem;

public class LockWinches extends CommandBase {
  /**
   * Creates a new LockWinches.
   */

  private final ClimbSubsystem cs;

  public LockWinches() {
    // Use addRequirements() here to declare subsystem dependencies.

    cs = Factory.getClimb();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!cs.getActivated()) {
      cs.setEnable(true);
      System.out.println(cs.getEnable());
    } else {
      cs.setEnable(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cs.setEnable(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
