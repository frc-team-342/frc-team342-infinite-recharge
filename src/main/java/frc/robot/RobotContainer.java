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
import edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.IntakeWithButton;
import frc.robot.commands.LaunchWithButton;
import frc.robot.commands.DriveWithJoystick;

import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  //remote controls 
  XboxController m_driverController = new XboxController(Constants.driver_joystick);
  private static Joystick joy;
  private static JoystickButton leftBumper; 
  private static JoystickButton rightBumper;
  private static JoystickButton xbox_A;  
  private static JoystickButton trigger; 

  //subsystems and commands
  private final DriveSystem driveSystem;
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final IntakeWithButton m_intakeWithButton = new IntakeWithButton();
  private final LaunchWithButton m_launchWithButton = new LaunchWithButton(); 
  

  //rotate to amgle 
  private Command RotateToAngle90 = new RotateToAngle(90.0);
  private Command RotateToAngle45 = new RotateToAngle(45.0);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {

    //establishes joysticks and buttons
    joy = new Joystick(1);
    //leftBumper = new JoystickButton(joy, Constants.LEFTBUMPER); 
    //rightBumper = new JoystickButton(joy, Constants.RIGHTBUMPER); 
    //xbox_A = new JoystickButton(joy, Constants.XBOX_A); 
    trigger = new JoystickButton(joy, Constants.TRIGGER); 
    driveSystem = Factory.getDrive();
    
    

    // Configure the button bindings
    configureButtonBindings();
  }
  public static Joystick getJoy(){
    return joy;
  }

  public static double driverAxis(){
    return joy.getRawAxis(Constants.driveYAxis);
  }

  

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //leftBumper.whileHeld(m_intakeWithButton);
    //rightBumper.whileHeld(m_launchWithButton); 
    //xbox_A.whenPressed(RotateToAngle90);
    trigger.whenPressed(RotateToAngle45); 
    
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
