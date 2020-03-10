/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RotateToAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class BeyBlade extends SequentialCommandGroup {
  /**
   * Creates a new BeyBlade.
   */
  public BeyBlade() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new AutoMove(0.0, 0.0, 0.45).withTimeout(13.0),
      new RotateToAngle(0.0),
      new AutoMove(0.0, 0.2, 0.0)
    );
  }
}