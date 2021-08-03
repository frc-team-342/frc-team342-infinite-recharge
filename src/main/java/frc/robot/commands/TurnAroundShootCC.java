// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnAroundShootCC extends SequentialCommandGroup {
  /** Creates a new TurnAroundShootCC. */
  public TurnAroundShootCC() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Drive Forwards, Turn Around, and Shoot (Counter-Clockwise)
      new AutoMove(0.45).withTimeout(0.8), 
      new RotateToAngle(-135.0),
      new AutoTarget().withTimeout(2.0),
      new LaunchWithButton().withTimeout(8.5)
    );
  }
}
