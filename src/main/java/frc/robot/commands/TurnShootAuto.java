// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Factory;

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
      // Rotate around to face approximately towards target
      new RunCommand(() -> {
        // Signum gets sign of angle: -1, 0, or 1
        Factory.getDrive().Drive(0.0, 0.0, Math.signum(angle) * 0.6);
      // Run for (|angle| / 100) seconds. Ex if angle is 135 run for 1.35 seconds
      }, Factory.getDrive()).withTimeout((Math.abs(angle) / 100)),
      // Attempts to target with limelight for a max of 2 seconds
      new AutoTarget().withTimeout(2.0),
      // Runs shooter with no end condition
      new LaunchWithButton()
    );
  }
}
