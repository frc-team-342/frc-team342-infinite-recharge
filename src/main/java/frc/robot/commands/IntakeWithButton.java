package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Factory;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeAndOutake;

public class IntakeWithButton extends CommandBase {
  /**
   * Intakes with left bumper
   */

  private final IntakeAndOutake intakeAndOutake;

  public IntakeWithButton() {
    intakeAndOutake = Factory.getIntakeOutake();

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intakeAndOutake.intake();
  }

  @Override
  public void end(boolean interrupted) {
    intakeAndOutake.intakeStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
