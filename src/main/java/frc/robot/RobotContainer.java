/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import frc.robot.subsystems.JetsonSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*import frc.robot.commands.RotateToAngle;
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
import frc.robot.commands.LockWinches;*/

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

  private static JoystickButton gyrozeroer;
  private static JoystickButton fieldtoggle;
  private static JoystickButton pidtoggle;
  private static JoystickButton toggleSlow;
  private static JoystickButton toggleTurbo;

  private static Joystick colorGetter;
  private static JoystickButton colorBtn;
  private static Joystick tele;
  private static JoystickButton winchLock;
  private static JoystickButton winchActivate;
  /*private final DriveWithJoystick driveWithJoystick;
  private final DriveWithPercent driveWithPercent;
  private final DriveSystem driveSystem;
  private final Autonomous auto = new Autonomous();


  private final IntakeWithButton m_intakeWithButton = new IntakeWithButton();
  private final LaunchWithButton m_launchWithButton = new LaunchWithButton(); */
  

  //rotate to amgle 
  /*private Command RotateToAngle90 = new RotateToAngle(90.0);
  private Command RotateToAngle45 = new RotateToAngle(45.0);
  private final JetsonSubsystem jetson = new JetsonSubsystem();
  private final ControlPanelSubsystem control;
  private final ChangeColor changeColor;
  private final LockWinches lockWinch;
  private final ActivateWinches activateWinches;

  private final ZeroGyro zero = new ZeroGyro();
  private final ToggleFieldOriented togglefield = new ToggleFieldOriented();
  private final TogglePID pid = new TogglePID();
  private final ToggleSlowMode slow = new ToggleSlowMode();
  private final ToggleTurboMode turbo = new ToggleTurboMode();*/

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
    //control = Factory.getControl();
    joy = new Joystick(Constants.driver_joystick);
    //driveSystem = Factory.getDrive();
    joy = new Joystick(Constants.driver_joystick);
    //driveWithJoystick = new DriveWithJoystick();
    //driveWithPercent = new DriveWithPercent();
    
    gyrozeroer = new JoystickButton(joy, Constants.zeroGyro);
    fieldtoggle = new JoystickButton(joy, Constants.fieldToggler);
    pidtoggle = new JoystickButton(joy, Constants.pidToggler);
    toggleSlow = new JoystickButton(joy, Constants.toggleSlow);
    toggleTurbo = new JoystickButton(joy, Constants.toggleTurbo);
    //changeColor = new ChangeColor();
    //lockWinch = new LockWinches();
    //activateWinches = new ActivateWinches();

    colorGetter = new Joystick(Constants.colorBtn);
    colorBtn = new JoystickButton(colorGetter, Constants.colorBtn);

    tele = new Joystick(1);
    winchLock = new JoystickButton(tele, 7);
    winchActivate = new JoystickButton(tele, 8);

    
    

    // Configure the button bindings
    configureButtonBindings();
  }
  public static Joystick getJoy(){
    return joy;
  }

  public static Joystick getTele() {
    return tele;
  }

  public static double driverAxis(){
    return joy.getRawAxis(Constants.driveYAxis);
  }

  /*public Command getDrive(){
    return driveWithJoystick; 
  }*/

  /*public Command getPercent(){
    return driveWithPercent;
  }*/

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*leftBumper.whileHeld(m_intakeWithButton);
    rightBumper.whileHeld(m_launchWithButton); 
    gyrozeroer.whenPressed(zero);
    fieldtoggle.whenPressed(togglefield);
    pidtoggle.whileHeld(pid);
    toggleSlow.whenPressed(slow);
    toggleTurbo.whenPressed(turbo);
    //leftBumper.whileHeld(m_intakeWithButton);
    //rightBumper.whileHeld(m_launchWithButton); 
    //xbox_A.whenPressed(RotateToAngle90);
    trigger.whenPressed(RotateToAngle45); 
    
    colorBtn.whenPressed(changeColor);
    winchLock.whileHeld(lockWinch);
    winchActivate.whileHeld(activateWinches);*/

  }

 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return auto;
  }*/

}
