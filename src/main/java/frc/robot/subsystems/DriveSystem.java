/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import com.revrobotics.CANSparkMax;

import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Robot;

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

    kP = 5e-5;
    kI = 1e-6;
    kD = 0;
    kIz = 0;
    kFF = 0.000156;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    setPID(motorLeft1);
    setPID(motorLeft2);
    setPID(motorRight1);
    setPID(motorRight2);

    motorLeft1.getEncoder().setPosition(0);

    encoderL1 = new CANEncoder(motorLeft1);
    encoderL2 = new CANEncoder(motorLeft2);
    encoderR1 = new CANEncoder(motorRight1);
    encoderR2 = new CANEncoder(motorRight2); 


    mecanumDrive = new MecanumDrive(motorLeft1, motorLeft2, motorRight1, motorRight2);

    NavX = new AHRS();

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

    } else if(isSlowMode) 
        mecanumDrive.driveCartesian((xSpeed * 0.8) / 2, (ySpeed * 0.8) / 2, (zRotation * 0.8) / 4, -NavX.getAngle());
      else if (isTurbo)
        mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation, -NavX.getAngle());
      else {
        mecanumDrive.driveCartesian(xSpeed * 0.8, ySpeed * 0.8, (zRotation * 0.8) / 2);
    }
  }

  public void zeroGyro() {
    NavX.zeroYaw();
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

  public void PercentOut(double yAxis) {
    motorLeft1.set(yAxis);
    motorLeft2.set(yAxis);
    motorRight1.set(yAxis);
    motorRight2.set(yAxis);
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

  public void autoStrafe(double distance) {
    motorLeft1.getEncoder().setPosition(0.0);
    double conversion = 1.0; // will change meters to encoders ticks
    double target = distance * conversion;
    double current = motorLeft1.getEncoder().getPosition();
    double kP = 2.0;
    mecanumDrive.driveCartesian(((target - current) * kP) / 300.0, 0.0, 0.0);
  }

  public void autoDrive(double distance) {
    motorLeft1.getEncoder().setPosition(0.0);
    double conversion = 1.0; // will change meters to encoder ticks
    double target = distance * conversion;
    double current = motorLeft1.getEncoder().getPosition();
    double kP = 2.0;
    mecanumDrive.driveCartesian(0.0, ((target - current) * kP) / 300, 0.0);
  }

  public void rotateByError(double Error) {
    accumError += Error;
    double kI = 1.0e-3;
    double kP = 4.0;
    mecanumDrive.driveCartesian(0.0, 0.0, ((Error * kP) + (accumError * kI)) / 300);
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

  public void stopDrive() {
    motorLeft1.stopMotor();
    motorLeft2.stopMotor();
    motorRight1.stopMotor();
    motorRight2.stopMotor();
  }

  @Override
  public void periodic() {
    mecanumDrive.feed();
    // This method will be called once per scheduler run
  }

  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoystick());
  }
}

