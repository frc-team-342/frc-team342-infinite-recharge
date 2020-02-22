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

    super(new AutoMove(0.0, 0.45, 0.0).withTimeout(1.35), new RotateToAngle(-70.0),
        new AutoMove(0.0, 0.5, 0.0).withTimeout(0.8), new RotateToAngle(-160.0),
        new AutoMove(0.0, 0.5, 0.0).withTimeout(0.4));

  }
}
