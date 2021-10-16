// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Drive off the initiation line, turn, and shoot with limelight targeting
 */
public class TurnShootAuto extends SequentialCommandGroup {
  
  /**
   * Instantiates the command but does not run until scheduled
   * 
   * @param angle the angle for the robot to turn to, clockwise positive, -180 to 180
   */
  public TurnShootAuto(double angle) {
    // Commands that are run by this command group
    addCommands(
      // Drive forwards at 0.45 speed for 0.8 seconds
      new AutoMove(0.45).withTimeout(0.8), 
      new RotateToAngle(angle),
      // Attempts to target with limelight for a max of 2 seconds
      new AutoTarget().withTimeout(2.0),
      // Runs shooter with no end condition
      new LaunchWithButton()
    );
  }
}
