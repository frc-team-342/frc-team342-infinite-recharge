/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.DriveWithPercent;
import frc.robot.commands.DriveWithTargeting;
import frc.robot.commands.IntakeWithButton;
import frc.robot.commands.ShootWithButton;
import frc.robot.commands.RotateToAngle;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeAndOutake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static JoystickButton gyrozeroer;
  private static Joystick joy;
  private static JoystickButton fieldtoggle;
  private static JoystickButton pidtoggle;
  private static JoystickButton toggleSlow;
  private static JoystickButton toggleTurbo;
  private static JoystickButton rotateToggle;
  private static JoystickButton toggleTarget;
  private static JoystickButton trigger;
  private static JoystickButton side;
  private static JoystickButton intakeReverse;

  private final DriveWithJoystick driveWithJoystick;
  private final DriveWithPercent driveWithPercent;
  private final DriveSystem driveSystem;
  private final IntakeAndOutake intakeAndOutake;

  private final RotateToAngle rotate = new RotateToAngle();
  private final IntakeWithButton m_intakeWithButton = new IntakeWithButton();
  private final ShootWithButton m_shooterWithButton = new ShootWithButton();

  private Command field;
  private Command slow;
  private Command turbo;
  private Command zero;
  private Command pid;
  private Command target;
  private Command reverse;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driveSystem = Factory.getDrive();
    joy = new Joystick(Constants.driver_joystick);
    driveWithJoystick = new DriveWithJoystick();
    driveWithPercent = new DriveWithPercent();
    intakeAndOutake = Factory.getIntakeOutake();

    gyrozeroer = new JoystickButton(joy, Constants.zeroGyro);
    fieldtoggle = new JoystickButton(joy, Constants.fieldToggler);
    // pidtoggle = new JoystickButton(joy, Constants.pidToggler);
    toggleSlow = new JoystickButton(joy, Constants.toggleSlow);
    toggleTurbo = new JoystickButton(joy, Constants.toggleTurbo);
    rotateToggle = new JoystickButton(joy, Constants.pidToggler);
    toggleTarget = new JoystickButton(joy, Constants.toggleTarget);
    intakeReverse = new JoystickButton(joy, Constants.toggleReversed);

    trigger = new JoystickButton(joy, Constants.TRIGGER);
    side = new JoystickButton(joy, Constants.SIDE);

    field = new InstantCommand(driveSystem::setFieldOriented, driveSystem);
    slow = new InstantCommand(driveSystem::setSlow, driveSystem);
    turbo = new InstantCommand(driveSystem::setTurbo, driveSystem);
    zero = new InstantCommand(driveSystem::zeroGyro, driveSystem);
    pid = new InstantCommand(driveSystem::setPIDLooped, driveSystem);
    target = new InstantCommand(driveSystem::toggleTargeting, driveSystem);
    reverse = new InstantCommand(intakeAndOutake::setReversed, intakeAndOutake);

    // Configure the button bindings
    configureButtonBindings();
  }

  public static Joystick getJoy() {
    return joy;
  }

  public static double driverAxis() {
    return joy.getRawAxis(Constants.driveYAxis);
  }

  public Command getDrive() {
    return driveWithJoystick;
  }

  public Command getPercent() {
    return driveWithPercent;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    gyrozeroer.whenPressed(zero);
    fieldtoggle.whenPressed(field);
    // pidtoggle.whenPressed(pid);
    toggleSlow.whenPressed(slow);
    toggleTurbo.whenPressed(turbo);
    rotateToggle.whileHeld(rotate);
    toggleTarget.whenPressed(target);
    intakeReverse.whenPressed(reverse);

    side.toggleWhenPressed(m_intakeWithButton);
    trigger.toggleWhenPressed(m_shooterWithButton);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}