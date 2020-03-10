/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LaunchWithButton;
import frc.robot.commands.RotateToAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonomousTrench6Ball extends SequentialCommandGroup {

  //THEORITICAL JAMES CAN'T CALL ME STUPID

  /**
   * Creates a new AutonomousTrench6Ball.
   */
  public AutonomousTrench6Ball() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new RotateToAngle(180),
      new AutoTarget().withTimeout(1.0),
      new LaunchWithButton().withTimeout(4.0),
      new RotateToAngle(180),
      new AutoIntake(0.0, 0.4, 0.0).withTimeout(2.5),
      new RotateToAngle(-160),
      new AutoTarget().withTimeout(1.0),
      new LaunchWithButton().withTimeout(6.0)
    );
  }
}