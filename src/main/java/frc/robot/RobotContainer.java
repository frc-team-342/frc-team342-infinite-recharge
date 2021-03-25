/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.commands.RotateToAngle;
import frc.robot.commands.IntakeWithButton;
import frc.robot.commands.LaunchWithButton;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.subsystems.DriveSystem;
import frc.robot.commands.ActivateWinches;
import frc.robot.commands.Autonomous;
import frc.robot.commands.ChangeColor;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;

import frc.robot.commands.ActivateTelescopes;

import frc.robot.commands.DriveWithTargeting;
import frc.robot.commands.LockWinches;
import frc.robot.commands.ManualControlPanel;
import frc.robot.commands.MoveArm;
import frc.robot.commands.ReverseIntake;
import frc.robot.subsystems.IntakeAndOutake;



/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  private final DriveSystem driveSystem;
  private final ClimbSubsystem climb;

  PhotonCamera camera; // ok

  // Driver controller
  private static Joystick driver; // port 0

  private JoystickButton driver_autoAlignBtn; // button 1
  private JoystickButton driver_fieldOrientBtn; // button 2
  private JoystickButton driver_turboBtn; // button 5
  private JoystickButton driver_slowBtn;
  private JoystickButton driver_zeroBtn;
  private JoystickButton driver_reverseBtn;

  private Command driver_autoAlign;
  private Command driver_fieldOrient;
  private Command driver_turbo;
  private Command driver_slow;
  private Command driver_zero;
  private Command driver_reverse;

  // Operator controller
  private static XboxController operator;

  private JoystickButton op_launchBtn;
  private JoystickButton op_slowBtn;
  private JoystickButton op_intakeBtn;
  private JoystickButton op_lockWinchBtn;
  private JoystickButton op_runWinchBtn; 
  private JoystickButton op_telescopesBtn;
  private JoystickButton op_reverseBtn;
  private JoystickButton op_controlarmBtn;
  private JoystickButton op_manual_wheelBtn;
  private JoystickButton op_reverse_teleBtn;

  private Command op_launch;
  private Command op_slow;
  private Command op_intake;
  private Command op_lockWinch;
  private Command op_runWinch;
  private Command op_telescopes;
  private Command op_reverse;
  private Command op_controlarm;
  private Command op_manual_wheel;
  private Command op_reverse_tele;

  private Command field;
  private Command slow;
  private Command turbo;
  private Command zero;
  private Command pid;
  private Command target;

  // Autonomous
  private Command auto;
  private Trajectory trajectory;
  private TrajectoryConfig config;
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driveSystem = Factory.getDrive();
    climb = Factory.getClimb();

    // Driver controller
    driver = new Joystick(Constants.DRIVER_CONTROLLER);

    
    driver_autoAlignBtn = new JoystickButton(driver, Constants.DRIVER_AUTO_ALIGN);
    driver_fieldOrientBtn = new JoystickButton(driver, Constants.DRIVER_FIELD_ORIENT);
    driver_turboBtn = new JoystickButton(driver, Constants.DRIVER_TURBO);
    driver_slowBtn = new JoystickButton(driver, Constants.DRIVER_SLOW);
    driver_zeroBtn = new JoystickButton(driver, Constants.DRIVER_ZERO);
    driver_reverseBtn = new JoystickButton(driver, Constants.DRIVER_REVERSE);

    driver_autoAlign = new InstantCommand(driveSystem::toggleTargeting, driveSystem);
    driver_fieldOrient = new InstantCommand(driveSystem::setFieldOriented, driveSystem);
    driver_turbo = new InstantCommand(driveSystem::setTurbo, driveSystem);
    driver_slow = new InstantCommand(driveSystem::setSlow, driveSystem);
    driver_zero = new InstantCommand(driveSystem::zeroGyro, driveSystem);
    driver_reverse = new ReverseIntake();
    
    driver_autoAlignBtn = new JoystickButton(driver, Constants.DRIVER_AUTO_ALIGN);
    driver_fieldOrientBtn = new JoystickButton(driver, Constants.DRIVER_FIELD_ORIENT);
    driver_turboBtn = new JoystickButton(driver, Constants.DRIVER_TURBO);




    // Operator controller
    operator = new XboxController(Constants.OPERATOR_CONTROLLER);

    op_launchBtn = new JoystickButton(operator, Constants.OP_LAUNCH);
    op_slowBtn = new JoystickButton(operator, Constants.OP_SLOW);
    op_intakeBtn = new JoystickButton(operator, Constants.OP_INTAKE);

    op_lockWinchBtn = new JoystickButton(operator, Constants.OP_LOCK_WINCH);
    op_runWinchBtn = new JoystickButton(operator, Constants.OP_RUN_WINCH);
    op_telescopesBtn = new JoystickButton(operator, Constants.OP_TELESCOPES);
    op_reverseBtn = new JoystickButton(operator, Constants.OP_REVERSE);
    op_controlarmBtn = new JoystickButton(operator, Constants.OP_CONTROL_ARM);
    op_manual_wheelBtn = new JoystickButton(operator, Constants.OP_CONTROL_RIGHT);
    op_reverse_teleBtn = new JoystickButton(operator, Constants.OP_REVERSE_TELE);
  
    op_launch = new LaunchWithButton();
    op_slow = new InstantCommand(driveSystem::setSlow, driveSystem);

    op_lockWinchBtn = new JoystickButton(operator, Constants.OP_LOCK_WINCH);
    op_runWinchBtn = new JoystickButton(operator, Constants.OP_RUN_WINCH);

    //op_slow = new ToggleSlowMode();

    op_intake = new IntakeWithButton();
    op_lockWinch = new LockWinches();
    op_runWinch = new ActivateWinches();
    op_telescopes = new ActivateTelescopes();
    op_reverse = new ReverseIntake();
    op_controlarm = new MoveArm();
    op_manual_wheel = new ManualControlPanel();
    op_reverse_tele = new InstantCommand(climb::setReverse, climb);

    // Autonomous
    auto = new Autonomous();

    configureButtonBindings();

    camera = new PhotonCamera("camera1");
  }

  public static Joystick getJoy(){
    return driver;
  }

  public static XboxController getOperator() {
    return operator;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver button bindings
    driver_autoAlignBtn.whenPressed(driver_autoAlign);
    driver_fieldOrientBtn.whenPressed(driver_fieldOrient);
    driver_turboBtn.whenPressed(driver_turbo);
    driver_slowBtn.whenPressed(driver_slow);
    driver_zeroBtn.whenPressed(driver_zero);
    driver_reverseBtn.whileHeld(driver_reverse);

    // Operator button bindings
    op_launchBtn.toggleWhenPressed(op_launch);
    op_slowBtn.whenPressed(op_slow);
    op_intakeBtn.toggleWhenPressed(op_intake);
    op_lockWinchBtn.whenPressed(op_lockWinch);
    op_runWinchBtn.whileHeld(op_runWinch);
    op_telescopesBtn.whileHeld(op_telescopes);
    op_reverseBtn.whileHeld(op_reverse);
    op_controlarmBtn.toggleWhenPressed(op_controlarm);
    op_manual_wheelBtn.whileHeld(op_manual_wheel);
    op_reverse_teleBtn.whenPressed(op_reverse_tele);
  }

  /**
   * Performs know calculation on given nav point to convert it from meters to field points in meters and corrects distances. 
   * Calculation was found by finding the inverse of equation derived from distance tests.
   * Due to the calculation containing square root mathematics, signum logic is applied to make sure negative nav points can still be calculated.
   * @param navpoint
   */
  public double getNavPointVertical(double navpoint){
    if(navpoint == 0.0){
      return 0;
    }
    else if(Math.signum(navpoint) == -1.0){
      //return -navpoint * Constants.fieldUnitsToMeters;
      return (-navpoint * Constants.fieldUnitsToMeters) - Constants.centerRobotToIntakeMeters;
    }
    else{
      //return navpoint * Constants.fieldUnitsToMeters;
      return (navpoint * Constants.fieldUnitsToMeters) + Constants.centerRobotToIntakeMeters;
      
    }
  }

  /**
   * Performs know calculation on given nav point to convert it from meters to field points in meters and corrects distances. 
   * Calculation was found by finding the inverse of equation derived from distance tests.
   * Returns the calculated nav point negated due to the trajectory point for the horizontal being naturally inverted by the generator.
   * @param navpoint
   */
  public double getNavPointHorizontal(double navpoint){
    if(navpoint != 0.0){
      return -navpoint * Constants.fieldUnitsToMeters;
    }
    else
      return 0.0;
  }

  public Trajectory.State getSample(){
    return trajectory.sample(trajectory.getTotalTimeSeconds());
  }

  public double centerRobotToIntakeX(double angle){
    return Constants.centerRobotToIntakeMeters * Math.cos(angle);
  }

  public double centerRobotToIntakeY(double angle){
    return Constants.centerRobotToIntakeMeters * Math.sin(angle);
  }

  public void redPathA(){
    double firstPointAngle = 0.0;
    double secondPointAngle = -0.470255442341;
    double thirdPointAngle = 1.25571844584;

    trajectory = TrajectoryGenerator.generateTrajectory(
      // The starting end point of the trajectory path
      new Pose2d(getNavPointVertical(0.0), getNavPointHorizontal(0.0), new Rotation2d(0)), 
      List.of(
        // Here is where you add interior waypoints
        // First point in the translation is the vertical position and second is the horizontal position
        new Translation2d(getNavPointVertical(2.0) - centerRobotToIntakeX(firstPointAngle), getNavPointHorizontal(0.0) - centerRobotToIntakeY(firstPointAngle)),
        new Translation2d(getNavPointVertical(4.0) - centerRobotToIntakeX(secondPointAngle), getNavPointHorizontal(1.0) - centerRobotToIntakeY(secondPointAngle)),
        new Translation2d(getNavPointVertical(5.0) - centerRobotToIntakeX(thirdPointAngle), getNavPointHorizontal(-2.0) - centerRobotToIntakeY(thirdPointAngle))
      ), 
      // The final end point of the trajectory path
      new Pose2d(getNavPointVertical(10.0), getNavPointHorizontal(-2.0), new Rotation2d(0)), 
      config
    ); 
  }

  public void redPathB(){
    trajectory = TrajectoryGenerator.generateTrajectory(
      // The starting end point of the trajectory path
      new Pose2d(getNavPointVertical(0.0), getNavPointHorizontal(0.0), new Rotation2d(0)), 
      List.of(
        // Here is where you add interior waypoints
        // First point in the translation is the vertical position and second is the horizontal position
        new Translation2d(getNavPointVertical(2.5), getNavPointHorizontal(0.0)),
        new Translation2d(getNavPointVertical(4.0), getNavPointHorizontal(2.0)),
        new Translation2d(getNavPointVertical(6.0), getNavPointHorizontal(0.0)),
        new Translation2d(getNavPointVertical(7.0), getNavPointHorizontal(-1.0))
      ), 
      // The final end point of the trajectory path
      new Pose2d(getNavPointVertical(10.75), getNavPointHorizontal(-1.0), new Rotation2d(0)), 
      config
    ); 
  }

  public void bluePathA(){
    trajectory = TrajectoryGenerator.generateTrajectory(
      // The starting end point of the trajectory path
      new Pose2d(getNavPointVertical(0.0), getNavPointHorizontal(0.0), new Rotation2d(0)), 
      List.of(
        // Here is where you add interior waypoints
        // First point in the translation is the vertical position and second is the horizontal position
        new Translation2d(getNavPointVertical(5.0), getNavPointHorizontal(0.0)),
        new Translation2d(getNavPointVertical(6.0), getNavPointHorizontal(-3.0)),
        new Translation2d(getNavPointVertical(8.0), getNavPointHorizontal(-2.0))
      ), 
      // The final end point of the trajectory path
      new Pose2d(getNavPointVertical(10.5), getNavPointHorizontal(-2.0), new Rotation2d(0)), 
      config
    );
  }

  public void bluePathB(){
    trajectory = TrajectoryGenerator.generateTrajectory(
      // The starting end point of the trajectory path
      new Pose2d(getNavPointVertical(0.0), getNavPointHorizontal(0.0), new Rotation2d(0)), 
      List.of(
        // Here is where you add interior waypoints
        // First point in the translation is the vertical position and second is the horizontal position
        new Translation2d(getNavPointVertical(5.0), getNavPointHorizontal(0.0)),
        new Translation2d(getNavPointVertical(7.0), getNavPointHorizontal(-2.0)),
        new Translation2d(getNavPointVertical(9.0), getNavPointHorizontal(0.0))
      ), 
      // The final end point of the trajectory path
      new Pose2d(getNavPointVertical(10.5), getNavPointHorizontal(1.0), new Rotation2d(0)), 
      config
    );
  }

  //Uses the angle of the Powercells to identify which path to use
  //Does stuff *thumbs up*
  public void galacticSearchWithPC() {
    PhotonTrackedTarget target = camera.getLatestResult().getTargets().get(0);

    if (camera.getLatestResult().hasTargets()) {
      double angle = target.getYaw();

      if (angle < -0.81 && angle > -6.81) {
        //redPathA();
        System.out.println("red path a");
      }
      else if (angle < -7.75 && angle > -13.75) {
        //redPathB();
        System.out.println("red path b");
      }
      else if (angle < 6.36 && angle > 0.36) {
        //bluePathA();
        System.out.println("blue path a");
      } else {
        // bluePathB();
        System.out.println("blue path BBB has target");
      }
    } else {
      //bluePathB();
      System.out.println("blu path BBBBB no targetsss");
    }
  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Sets a voltage constraint so the trajectory never commands the robot to go faster than it is capable with its given voltage supply
    DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        Constants.ksVolts, 
        Constants.kvVoltsSecondsPerMeter, 
        Constants.kaVoltsSecondsSquaredPerMeter
      ), 
      Constants.kDifferentialKinematics, 
      10 // magic numbers babey
    );

    // Wraps together all of the path constraints
    config = new TrajectoryConfig(
      Constants.kMaxSpeedMetersPerSecond, 
      Constants.kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(Constants.kDifferentialKinematics)
    .addConstraint(voltageConstraint);
    
    //Autonomous Goes Here
    galacticSearchWithPC();

    RamseteCommand ramsete = new RamseteCommand(
      trajectory, 
      driveSystem::getPose2d, 
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), 
      new SimpleMotorFeedforward(
        Constants.ksVolts, 
        Constants.kvVoltsSecondsPerMeter, 
        Constants.kaVoltsSecondsSquaredPerMeter
      ),
      Constants.kDifferentialKinematics, 
      driveSystem::getDifferentialWheelSpeeds, 
      new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel), 
      new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel), 
      driveSystem::differentialDriveVolts, 
      driveSystem
    );
    
    driveSystem.resetOdometry(trajectory.getInitialPose());

    // Command group that allows us to run to commands simultaneously. 
    // Applied for using power cell intake while performing autonomous trajectory
    return new ParallelRaceGroup(
      ramsete.andThen(() -> driveSystem.differentialDriveVolts(0, 0)),
      new RunCommand(
        () -> {
          Factory.getIntakeOutake().intake();
        },
        Factory.getIntakeOutake()
      )
    );
  }
}