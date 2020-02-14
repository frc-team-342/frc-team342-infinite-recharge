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
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.ToggleFieldOriented;
import frc.robot.commands.TogglePID;
import frc.robot.commands.ToggleSlowMode;
import frc.robot.commands.ToggleTurboMode;
import frc.robot.commands.ZeroGyro;
import frc.robot.subsystems.DriveSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static JoystickButton gyrozeroer;
  private static Joystick joy;
  private static JoystickButton fieldtoggle;
  private static JoystickButton pidtoggle;
  private static JoystickButton toggleSlow;
  private static JoystickButton toggleTurbo;
  private static JoystickButton rotateToggle;

  private final DriveWithJoystick driveWithJoystick;
  private final DriveWithPercent driveWithPercent;
  private final DriveSystem driveSystem;

  private final DriveWithTargeting rotate = new DriveWithTargeting();
  private final ZeroGyro zero = new ZeroGyro();
  private final ToggleFieldOriented togglefield = new ToggleFieldOriented();
  private final TogglePID pid = new TogglePID();
  private final ToggleSlowMode slow = new ToggleSlowMode();
  private final ToggleTurboMode turbo = new ToggleTurboMode();



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driveSystem = Factory.getDrive();
    joy = new Joystick(Constants.driver_joystick);
    driveWithJoystick = new DriveWithJoystick();
    driveWithPercent = new DriveWithPercent();
    
    gyrozeroer = new JoystickButton(joy, Constants.zeroGyro);
    fieldtoggle = new JoystickButton(joy, Constants.fieldToggler);
    // pidtoggle = new JoystickButton(joy, Constants.pidToggler);
    toggleSlow = new JoystickButton(joy, Constants.toggleSlow);
    toggleTurbo = new JoystickButton(joy, Constants.toggleTurbo);
    rotateToggle = new JoystickButton(joy, Constants.pidToggler);
    
    
    
    // Configure the button bindings
    configureButtonBindings();
  }
  public static Joystick getJoy(){
    return joy;
  }

  public static double driverAxis(){
    return joy.getRawAxis(Constants.driveYAxis);
  }

  public Command getDrive(){
    return driveWithJoystick;
  }

  public Command getPercent(){
    return driveWithPercent;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    gyrozeroer.whenPressed(zero);
    fieldtoggle.whenPressed(togglefield);
    // pidtoggle.whenPressed(pid);
    toggleSlow.whenPressed(slow);
    toggleTurbo.whenPressed(turbo);
    rotateToggle.whileHeld(rotate);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
