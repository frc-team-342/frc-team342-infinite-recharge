/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous.
   */
  public Autonomous() {

    super(new AutoMove(0.0, 0.6, 0.0).withTimeout(0.7),
      new RotateToAngle(-90.0),
      new AutoMove(0.0, 0.6, 0.0).withTimeout(0.5),
      new RotateToAngle(-179.0),
      new AutoMove(0.0, 0.5, 0.0).withTimeout(0.4));

  }
}
