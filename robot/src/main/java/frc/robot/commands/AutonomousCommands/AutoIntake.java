package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Factory;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeAndOutake;

public class AutoIntake extends CommandBase {
  /**
   * Intakes with left bumper
   */

  private final IntakeAndOutake intakeAndOutake;
  private final DriveSystem drive;
  private double X, Y, Z;

  public AutoIntake(double x, double y, double z) {
    intakeAndOutake = Factory.getIntakeOutake();
    drive = Factory.getDrive();

    X = x;
    Y = y;
    Z = z;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intakeAndOutake.intake();
    drive.Drive(X, Y, Z);
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