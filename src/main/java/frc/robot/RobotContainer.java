/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.JetsonSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.RotateToAngle;
import frc.robot.commands.IntakeWithButton;
import frc.robot.commands.LaunchWithButton;
import frc.robot.commands.DriveWithJoystick;

import frc.robot.subsystems.DriveSystem;
import frc.robot.commands.Autonomous;
import frc.robot.commands.DriveWithPercent;
import frc.robot.commands.ToggleFieldOriented;
import frc.robot.commands.TogglePID;
import frc.robot.commands.ToggleSlowMode;
import frc.robot.commands.ToggleTurboMode;
import frc.robot.commands.ZeroGyro;
import frc.robot.commands.ChangeColor;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.commands.ActivateTelescopes;
import frc.robot.commands.ActivateWinches;
import frc.robot.commands.AngleWithLimelight;
import frc.robot.commands.LockWinches;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  // Driver controller
  private static Joystick driver; // port 0

  private JoystickButton driver_autoAlignBtn; // button 1
  private JoystickButton driver_fieldOrientBtn; // button 2
  private JoystickButton driver_turboBtn; // button 5

  private Command driver_autoAlign;
  private Command driver_fieldOrient;
  private Command driver_turbo;

  // Operator controller
  private static XboxController operator;

  private JoystickButton op_launchBtn; // button 4
  private JoystickButton op_slowBtn; // button 5
  private JoystickButton op_intakeBtn; // button 6
  private JoystickButton op_lockWinchBtn; // button 7
  private JoystickButton op_runWinchBtn; // button 8

  private Command op_launch;
  private Command op_slow;
  private Command op_intake;
  private Command op_lockWinch;
  private Command op_runWinch;

  // Autonomous
  private Command auto;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Driver controller
    driver = new Joystick(Constants.DRIVER_CONTROLLER);
    
    driver_autoAlignBtn = new JoystickButton(driver, Constants.DRIVER_AUTOALIGN);
    driver_fieldOrientBtn = new JoystickButton(driver, Constants.DRIVER_FIELDORIENT);
    driver_turboBtn = new JoystickButton(driver, Constants.DRIVER_TURBO);

    driver_autoAlign = new AngleWithLimelight();
    driver_fieldOrient = new ToggleFieldOriented();
    driver_turbo = new ToggleTurboMode();

    // Operator controller
    operator = new XboxController(Constants.OPERATOR_CONTROLLER);

    op_launchBtn = new JoystickButton(operator, Constants.OP_LAUNCH);
    op_slowBtn = new JoystickButton(operator, Constants.OP_SLOW);
    op_intakeBtn = new JoystickButton(operator, Constants.OP_INTAKE);
    op_lockWinchBtn = new JoystickButton(operator, Constants.OP_LOCKWINCH);
    op_runWinchBtn = new JoystickButton(operator, Constants.OP_RUNWINCH);

    op_launch = new LaunchWithButton();
    op_slow = new ToggleSlowMode();
    op_intake = new IntakeWithButton();
    op_lockWinch = new LockWinches();
    op_runWinch = new ActivateWinches();

    // Autonomous
    auto = new Autonomous();

    configureButtonBindings();
  }

  public static Joystick getJoy(){
    return driver;
  }

  public static XboxController getTele() {
    return operator;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver button bindings
    driver_autoAlignBtn.whenPressed(driver_autoAlign);
    driver_fieldOrientBtn.whenPressed(driver_fieldOrient);
    driver_turboBtn.whenPressed(driver_turbo);

    // Operator button bindings
    op_launchBtn.whenPressed(op_launch);
    op_slowBtn.whenPressed(op_slow);
    op_intakeBtn.whenPressed(op_intake);
    op_lockWinchBtn.whenPressed(op_lockWinch);
    op_runWinchBtn.whenPressed(op_runWinch);
  }

 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto;
  }

}
