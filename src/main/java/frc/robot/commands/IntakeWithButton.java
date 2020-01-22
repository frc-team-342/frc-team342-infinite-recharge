package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Factory;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSystem;

public class IntakeWithButton extends CommandBase {
  /**
   * Intakes with left bumper
   */

  private final IntakeSystem intakeSystem; 
  private final Joystick joy; 

  public IntakeWithButton() {

    intakeSystem = Factory.getIntake(); 
    joy = RobotContainer.getJoy(); 

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intakeSystem.intake(); 
  }

  @Override
  public void end(boolean interrupted) {
    intakeSystem.intakeStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
