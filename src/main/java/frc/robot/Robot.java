/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.ActivateTelescopes;
import frc.robot.commands.AutonomousCommands.AutonomousLine;
import frc.robot.commands.AutonomousCommands.AutonomousShoot;
import frc.robot.commands.AutonomousCommands.AutonomousTrench;
import frc.robot.commands.AutonomousCommands.AutonomousTrench5Ball;
import frc.robot.commands.AutonomousCommands.AutonomousTrench6Ball;
import frc.robot.commands.AutonomousCommands.BeyBlade;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.DriveWithTargeting;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeAndOutake;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;
  private Command driveWithJoy;
  private static DriveSystem driveSystem;
  private static IntakeAndOutake intakeAndOutake;
  private Command autoDrive;
  private Command driveWithTargeting;
  private LimelightSubsystem lime;

  private SendableChooser<Command> autoChoose;


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    
    
    driveSystem = Factory.getDrive();
    lime = Factory.getLimelight();
    intakeAndOutake = Factory.getIntakeOutake();

    driveWithJoy = new DriveWithJoystick();
    autoDrive = new AutonomousShoot();
    driveWithTargeting = new DriveWithTargeting();

    autoChoose = new SendableChooser<>();
    autoChoose.setDefaultOption("Turn and Shoot", new AutonomousShoot());
    autoChoose.addOption("Trench FIVE Ball", new AutonomousTrench5Ball());
    autoChoose.addOption("Trench SIX Ball", new AutonomousTrench6Ball());
    autoChoose.addOption("Drive Off Line", new AutonomousLine());
    autoChoose.addOption("Let It Rip", new BeyBlade());
    

    SmartDashboard.putData("Auto Chooser", autoChoose);
    
    // Commands

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    lime.visionOff();
    autoDrive = autoChoose.getSelected();
    driveSystem.zeroGyro();
    autoDrive.schedule();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    driveSystem.targetOff();
    lime.visionOff();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autoDrive != null) {
      autoDrive.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    intakeAndOutake.getSensors();

    //intakeAndOutake.getSensors();

    if (driveSystem.getTarget()){
      driveWithJoy.cancel();
      driveWithTargeting.schedule();
    }
    else{
      driveWithTargeting.cancel();
      driveWithJoy.schedule();
    }
      

    // driveWithPercent.schedule();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}