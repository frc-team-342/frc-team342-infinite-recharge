/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.commands.DriveWithJoystick;

public class DriveSystem extends SubsystemBase {
  private CANSparkMax motorRight1;
  private CANSparkMax motorRight2;
  private CANSparkMax motorLeft1;
  private CANSparkMax motorLeft2;

  public CANEncoder encoderL1;
  public CANEncoder encoderL2;
  public CANEncoder encoderR1;
  public CANEncoder encoderR2;

  private SpeedControllerGroup m_leftMotors;
  private SpeedControllerGroup m_rightMotors;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  private boolean isFieldOriented = false;
  private boolean isPID = false;
  private boolean isSlowMode = false;
  private boolean isTurbo = false;
  private boolean isTargeting = false;

  private static final double ramp_rate = 0.2;
  private static final double voltage_comp = 12.0;
  private static final int current_limit = 60;
  private double accumError = 0.0;

  private AHRS NavX;
  private MecanumDrive mecanumDrive;
  private MecanumDriveKinematics kDriveKinematics;
  private final MecanumDriveOdometry m_odometry;
  private final DifferentialDriveOdometry d_odometry;

  /**
   * Creates a new DriveSystem.
   */
  public DriveSystem(CANSparkMax motor4, CANSparkMax motor2, CANSparkMax motor3, CANSparkMax motor1) {
    motorRight1 = motor3;
    motorRight2 = motor1;
    motorLeft1 = motor4;
    motorLeft2 = motor2;

    // set inversion
    motorLeft1.setInverted(false);
    motorLeft2.setInverted(false);
    motorRight1.setInverted(false);
    motorRight2.setInverted(false);

    // set current limits
    motorLeft1.setSmartCurrentLimit(current_limit);
    motorLeft2.setSmartCurrentLimit(current_limit);
    motorRight1.setSmartCurrentLimit(current_limit);
    motorRight2.setSmartCurrentLimit(current_limit);

    // set voltage comp
    motorLeft1.enableVoltageCompensation(voltage_comp);
    motorLeft2.enableVoltageCompensation(voltage_comp);
    motorRight1.enableVoltageCompensation(voltage_comp);
    motorRight2.enableVoltageCompensation(voltage_comp);

    // set ramp rate
    motorLeft1.setOpenLoopRampRate(ramp_rate);
    motorLeft2.setOpenLoopRampRate(ramp_rate);
    motorRight1.setOpenLoopRampRate(ramp_rate);
    motorRight2.setOpenLoopRampRate(ramp_rate);

    kP = Constants.kPDriveVel;
    kI = 0.0;
    kD = Constants.kDDriveVel;
    kIz = 0.0;
    kFF = 0.0; //0.000156
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    setPID(motorLeft1);
    setPID(motorLeft2);
    setPID(motorRight1);
    setPID(motorRight2);

    encoderL1 = motorLeft1.getEncoder();
    encoderL2 = motorLeft2.getEncoder();
    encoderR1 = motorRight1.getEncoder();
    encoderR2 = motorRight2.getEncoder(); 

    // Motors controller groups to be used for driving the robot as a tank
    m_leftMotors = new SpeedControllerGroup(motorLeft1, motorLeft2);
    m_rightMotors = new SpeedControllerGroup(motorRight1, motorRight2);

    mecanumDrive = new MecanumDrive(motorLeft1, motorLeft2, motorRight1, motorRight2);
    NavX = new AHRS();
    
    // Encoders must be reset to 0 --> BEFORE <-- constructing odometry objects.
    zeroGyro();
    resetEncoders();
    m_odometry = new MecanumDriveOdometry(kDriveKinematics, NavX.getRotation2d());
    d_odometry = new DifferentialDriveOdometry(NavX.getRotation2d(), new Pose2d(0.0, 0.0, new Rotation2d()));
  }

  public void Drive(double xSpeed, double ySpeed, double zRotation) {
    mecanumDrive.feed();
    if (isFieldOriented == true) {
      if (isSlowMode == true)
        mecanumDrive.driveCartesian((xSpeed * 0.8) / 2, (ySpeed * 0.8) / 2, (zRotation * 0.8) / 4, -NavX.getAngle());
      else if (isTurbo == true)
        mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation, -NavX.getAngle());
      else
        mecanumDrive.driveCartesian(xSpeed, ySpeed, (zRotation) / 2, -NavX.getAngle());

    } else if (isPID == true) {
      double target = 70.0;
      double current = NavX.getAngle();
      double kP = 2.0;

      mecanumDrive.driveCartesian(0.0, 0.0, ((target - current) * kP) / 300);

    } else if(isSlowMode){ 
        mecanumDrive.driveCartesian((xSpeed * 0.8) / 2, (ySpeed * 0.8) / 2, (zRotation * 0.8) / 4/*, -NavX.getAngle()*/);
        isFieldOriented = false;
        isTurbo = false;
    } else if (isTurbo){
        mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation/*, -NavX.getAngle()*/);
        isFieldOriented = false;
        isSlowMode = false;
  }   else {
        mecanumDrive.driveCartesian(xSpeed * 0.8, ySpeed * 0.8, (zRotation * 0.8) / 2);
        isFieldOriented = false;
        isSlowMode = false;
        isTurbo = false;
    }
  }

  /** Returns the speed of the drive wheels in a DifferentialDriveWheelSpeeds data type to be used later in RAMSETE command for trajectory.
   *  Multiplies encoder RPM by circumference of wheel and then divides by gear ratio to get wheel speed per minute and divides by 60 to get wheel speed per second.
   */
  public MecanumDriveWheelSpeeds getMecanumWheelSpeeds(){
    return new MecanumDriveWheelSpeeds(
      (encoderL1.getVelocity() * (Math.PI * Constants.wheelDiameterInMeters)) / (60 * Constants.gearRatio),
      (encoderR1.getVelocity() * (Math.PI * Constants.wheelDiameterInMeters)) / (60 * Constants.gearRatio),
      (encoderL2.getVelocity() * (Math.PI * Constants.wheelDiameterInMeters)) / (60 * Constants.gearRatio),
      (encoderR2.getVelocity() * (Math.PI * Constants.wheelDiameterInMeters)) / (60 * Constants.gearRatio)
    );
  }

  /** Returns the speed of the drive wheels in a DifferentialDriveWheelSpeeds data type to be used later in RAMSETE command for trajectory.
   *  Multiplies encoder RPM by circumference of wheel and then divides by gear ratio to get wheel speed per minute and divides by 60 to get wheel speed per second.
   */
  public DifferentialDriveWheelSpeeds getDifferentialWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      (encoderL2.getVelocity() * (Math.PI * Constants.wheelDiameterInMeters)) / (60 * Constants.gearRatio), // uhh oof ouch owie 
      (encoderR2.getVelocity() * (Math.PI * Constants.wheelDiameterInMeters)) / (60 * Constants.gearRatio)
    );
  }

  public void zeroGyro() {
    NavX.reset();
  }

  /** Returns the heading of the NavX negated so it is clockwise positive instead of default counter-clockwise positive */
  public double getHeading(){
    return NavX.getRotation2d().getDegrees();
  }

  public Pose2d getPose2d(){
    return d_odometry.getPoseMeters();
  }

  /** Sets the voltage of the left and ride motor controller groups for differential drive with voltage and feeds watchdog.
   * @param leftVolts voltage applied to left side of drive train
   * @param rightVolts voltage applied to right side of the drive train
   */
  public void differentialDriveVolts(double leftVolts, double rightVolts){
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    mecanumDrive.feed();
  }

  /** Resets the encoders and resets the pose of the odometry object to the pose parameter.
   * @param pose pose for the robot to be reset to.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, NavX.getRotation2d());
    d_odometry.resetPosition(pose, NavX.getRotation2d());
  }

  /** Sets all 4 drive train encoders to 0 */
  public void resetEncoders(){
    encoderL1.setPosition(0);
    encoderL2.setPosition(0);
    encoderR1.setPosition(0);
    encoderR2.setPosition(0);
  }

  /** Gets the distance the wheel has traveled by dividing the encoders position by gear ratio and multiplying by circumference of wheel
   * @param encoder encoder to get the wheel distance from.
  */
  public double getDistance(CANEncoder encoder){
    return (encoder.getPosition() / Constants.gearRatio) * (Math.PI * Constants.wheelDiameterInMeters);
  }

  public void setPID(CANSparkMax motor) {
    CANPIDController pid = motor.getPIDController();

    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    pid.setIZone(kIz);
    pid.setFF(kFF);
    pid.setOutputRange(kMinOutput, kMaxOutput);

    motor.setSmartCurrentLimit(current_limit);
    motor.enableVoltageCompensation(voltage_comp);
  }

  public void setPIDLooped() {
    isPID = !isPID;

    SmartDashboard.putBoolean("Is PID", isPID);
  }

  public void setFieldOriented() {
    isFieldOriented = !isFieldOriented;
    SmartDashboard.putBoolean("Is Field Oriented", isFieldOriented);
  }

  public void setSlow() {
    isSlowMode = !isSlowMode;
    if (isTurbo)
      isTurbo = false;
    SmartDashboard.putBoolean("Is Slow", isSlowMode);
  }

  public void setTurbo() {
    isTurbo = !isTurbo;
    if (isSlowMode)
      isSlowMode = false;
    SmartDashboard.putBoolean("Is Turbo", isTurbo);
  }

  public void autoRotate(double angle) {
    double target = angle;
    double current = NavX.getAngle();
    double kP = 2.0;
    mecanumDrive.driveCartesian(0.0, 0.0, ((target - current) * kP) / 300);
  }

  // PID Loop for driving the robot while also targeting with limelgiht
  public void driveWithTargeting(double x, double y, double Error) {
    accumError += Error;
    double kI = 1.0e-3;
    double kP = 4.0;
    double targetError = ((Error * kP) + (accumError * kI)) / 300;
    mecanumDrive.driveCartesian(x / 2, y / 2, targetError);

  }

  // Toggle for limelight targeting
  public void toggleTargeting() {
    isTargeting = !isTargeting;
    SmartDashboard.putBoolean("Is Targeting", isTargeting);
  }


  // Set off toggle for robot start so targeting is always off unless turned on
  public void targetOff() {
    isTargeting = false;
    SmartDashboard.putBoolean("Is Targeting", isTargeting);
  }

  public boolean getTarget() {
    return isTargeting;
  }

  public double getGyro() {
    return NavX.getAngle();
  }

  public void errorAccumReset() {
    System.out.println("h");
    accumError = 0.0;
  }

  public double getAccumError() {
    return accumError;
  }

  /** Returns the average of the total wheel distance traveled by the left and right side of the robot */
  public double getAvgEncoderDistance(){
    return (getDistance(encoderL1) + (-getDistance(encoderR1))) / 2;
  }

  /** Stops all 4 drive system motors */
  public void stopDrive() {
    motorLeft1.stopMotor();
    motorLeft2.stopMotor();
    motorRight1.stopMotor();
    motorRight2.stopMotor();
  }

  @Override
  public void periodic() {
    mecanumDrive.feed(); // Feeding the Watchdog with the mecanumDrive object so it doesn't cause startCompetition() errors and cause drive to not work.

    //m_odometry.update(NavX.getRotation2d(), getWheelSpeeds());
    Translation2d translation = d_odometry.getPoseMeters().getTranslation(); // Getting the pose translation of the odometry object so it can be displayed on SmartDashboard.
    d_odometry.update(NavX.getRotation2d(), getDistance(encoderL1), getDistance(encoderR1)); // updates the odometry object so it can accurately track robot position for trajectory.

    SmartDashboard.putNumber("Avg Encoder Distance: ", getAvgEncoderDistance());
    SmartDashboard.putNumber("Left Encoder Distance: ", getDistance(encoderL1));
    SmartDashboard.putNumber("Right Encoder Distance: ", -getDistance(encoderR1));
    SmartDashboard.putNumber("Translation X: ", translation.getX());
    SmartDashboard.putNumber("Translation Y: ", translation.getY());
  
    // This method will be called once per scheduler run
  }

  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoystick());
  }
}

