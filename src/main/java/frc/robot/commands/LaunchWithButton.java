/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeAndOutake;
import frc.robot.Factory;
import frc.robot.RobotContainer;

public class LaunchWithButton extends CommandBase {
  /**
   * Will shooter the powercells
   */

  private final IntakeAndOutake intakeAndOutake;
  private final Joystick joy;

  public LaunchWithButton() {
    intakeAndOutake = Factory.getIntakeOutake();
    joy = RobotContainer.getJoy();
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intakeAndOutake.outake();
  }

  @Override
  public void end(boolean interrupted) {

    intakeAndOutake.shooterStop();

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}