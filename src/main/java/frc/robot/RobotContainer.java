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
import frc.robot.commands.ChangeColor;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.commands.ChangeColor;
import frc.robot.subsystems.DriveSystem;
import frc.robot.commands.ActivateTelescopes;
import frc.robot.commands.ActivateWinches;
import frc.robot.commands.LockWinches;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static Joystick joy;
  private static Joystick colorGetter;
  private static JoystickButton colorBtn;
  private static Joystick tele;
  private static JoystickButton winchLock;
  private static JoystickButton winchActivate;
  private final DriveWithJoystick driveWithJoystick;
  private final DriveSystem driveSystem;

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final JetsonSubsystem jetson = new JetsonSubsystem();
  private final ControlPanelSubsystem control;
  private final ChangeColor changeColor;
  private final LockWinches lockWinch;
  private final ActivateWinches activateWinches;


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    control = Factory.getControl();
    joy = new Joystick(Constants.driver_joystick);
    driveSystem = Factory.getDrive();
    driveWithJoystick = new DriveWithJoystick();
    changeColor = new ChangeColor();
    lockWinch = new LockWinches();
    activateWinches = new ActivateWinches();

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

  public Command getDrive(){
    return driveWithJoystick;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    colorBtn.whenPressed(changeColor);
    winchLock.whileHeld(lockWinch);
    winchActivate.whileHeld(activateWinches);

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
