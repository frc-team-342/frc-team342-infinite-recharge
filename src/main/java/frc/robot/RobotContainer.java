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
<<<<<<< HEAD
import edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeWithButton;
import frc.robot.commands.LaunchWithButton;
import frc.robot.commands.DriveWithJoystick;

import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.LaunchSubsystem;
=======
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.DriveWithPercent;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ToggleFieldOriented;
import frc.robot.commands.TogglePID;
import frc.robot.commands.ToggleSlowMode;
import frc.robot.commands.ToggleTurboMode;
import frc.robot.commands.ZeroGyro;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
>>>>>>> 9c30ef15ff90410c99a339b8cc146eac5e6949f8

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
<<<<<<< HEAD

  //remote controls 
  XboxController m_driverController = new XboxController(Constants.driver_joystick);
  private static Joystick joy;
  private static JoystickButton leftBumper; 
  private static JoystickButton rightBumper; 

  //subsystems and commands
=======
  private static JoystickButton gyrozeroer;
  private static Joystick joy;
  private static JoystickButton fieldtoggle;
  private static JoystickButton pidtoggle;
  private static JoystickButton toggleSlow;
  private static JoystickButton toggleTurbo;

>>>>>>> 9c30ef15ff90410c99a339b8cc146eac5e6949f8
  private final DriveWithJoystick driveWithJoystick;
  private final DriveWithPercent driveWithPercent;
  private final DriveSystem driveSystem;

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
<<<<<<< HEAD
  private final IntakeWithButton m_intakeWithButton = new IntakeWithButton();
  private final LaunchWithButton m_launchWithButton = new LaunchWithButton(); 
=======
  private final ZeroGyro zero = new ZeroGyro();
  private final ToggleFieldOriented togglefield = new ToggleFieldOriented();
  private final TogglePID pid = new TogglePID();
  private final ToggleSlowMode slow = new ToggleSlowMode();
  private final ToggleTurboMode turbo = new ToggleTurboMode();

>>>>>>> 9c30ef15ff90410c99a339b8cc146eac5e6949f8


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
<<<<<<< HEAD

    //establishes joysticks and buttons
    leftBumper = new JoystickButton(joy, Constants.LEFTBUMPER); 
    rightBumper = new JoystickButton(joy, Constants.RIGHTBUMPER); 
=======
>>>>>>> 9c30ef15ff90410c99a339b8cc146eac5e6949f8
    driveSystem = Factory.getDrive();
    joy = new Joystick(Constants.driver_joystick);
    driveWithJoystick = new DriveWithJoystick();
    driveWithPercent = new DriveWithPercent();
    
    gyrozeroer = new JoystickButton(joy, Constants.zeroGyro);
    fieldtoggle = new JoystickButton(joy, Constants.fieldToggler);
    pidtoggle = new JoystickButton(joy, Constants.pidToggler);
    toggleSlow = new JoystickButton(joy, Constants.toggleSlow);
    toggleTurbo = new JoystickButton(joy, Constants.toggleTurbo);
    
    

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
<<<<<<< HEAD
    leftBumper.whileHeld(m_intakeWithButton);
    rightBumper.whileHeld(m_launchWithButton); 
=======
    gyrozeroer.whenPressed(zero);
    fieldtoggle.whenPressed(togglefield);
    pidtoggle.whileHeld(pid);
    toggleSlow.whenPressed(slow);
    toggleTurbo.whenPressed(turbo);
>>>>>>> 9c30ef15ff90410c99a339b8cc146eac5e6949f8
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
