/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveWithJoystick;

public class DriveSystem extends SubsystemBase {

  private Joystick joy = new Joystick(1);
  private CANSparkMax motorRight1;
  private CANSparkMax motorRight2;
  private CANSparkMax motorLeft1; 
  private CANSparkMax motorLeft2;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  private boolean isFieldOriented = false;
  private boolean isPID = false;
  private boolean isSlowMode = false;
  private boolean isTurbo = false;
  
  private static final double ramp_rate = 0.2;
  private static final double voltage_comp = 12.0;
  private static final int current_limit = 50;

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

    //set ramp rate
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

    mecanumDrive = new MecanumDrive(motorLeft1, motorLeft2, motorRight1, motorRight2);

    NavX = new AHRS();
  }
  
  public void Drive(double xSpeed, double ySpeed, double zRotation) {
    mecanumDrive.feed();
    if(isFieldOriented == true){
      if(isSlowMode == true)
       mecanumDrive.driveCartesian((xSpeed*0.8)/2, (ySpeed*0.8)/2, (zRotation*0.8)/4, -NavX.getAngle());
      else if(isTurbo == true)
        mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation, -NavX.getAngle());
      else
        mecanumDrive.driveCartesian(xSpeed, ySpeed, (zRotation)/2, -NavX.getAngle());
    }
    else if(isPID == true){
      SmartDashboard.putNumber("Slider: ", joy.getRawAxis(3));
      double target = 70.0;
      double current = NavX.getAngle();
      double kP = 2.0;

      mecanumDrive.driveCartesian(0.0, 0.0, ((target-current)*kP)/100);

    }   
    else
      if(isSlowMode == true)
       mecanumDrive.driveCartesian((xSpeed*0.8)/2, (ySpeed*0.8)/2, (zRotation*0.8)/4);
      else if(isTurbo == true)
       mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation);
      else{
        mecanumDrive.driveCartesian(xSpeed*0.8, ySpeed*0.8, (zRotation*0.8)/2);
      }
  }
  
  public void zeroGyro(){
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

  protected static void normalize(double wheelSpeeds[]) {
    double maxMagnitude = Math.abs(wheelSpeeds[0]);
    int i;
    for (i=1; i<4; i++) {
        double temp = Math.abs(wheelSpeeds[i]);
        if (maxMagnitude < temp) maxMagnitude = temp;
    }
    if (maxMagnitude > 1.0) {
        for (i=0; i<4; i++) {
            wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
        }
    }
  }

  public void PercentOut(double yAxis){
    motorLeft1.set(yAxis);
    motorLeft2.set(yAxis);
    motorRight1.set(yAxis);
    motorRight2.set(yAxis);
  }
  public void setPIDLooped(boolean bool){
    isPID = bool;
  }
  public boolean getPIDLooped(){
    return isPID;
  }

  public void setFieldOriented(boolean bool){
    isFieldOriented = bool;
  }
  public boolean getFieldOriented(){
    return isFieldOriented;
  }

  public void setSlow(boolean slow){
    isSlowMode = slow;
  }
  public boolean getSlow(){
    return isSlowMode;
  }

  public void setTurbo(boolean turbo){
    isTurbo = turbo;
  }
  public boolean getTurbo(){
    return isTurbo;
  }

  public void autoDrive(double xSpeed, double ySpeed, double zRotation){
    mecanumDrive.driveCartesian(xSpeed*0.8, ySpeed*0.8, (zRotation*0.8)/2, -NavX.getAngle());
  }
  public void autoRotate(double angle){
    double target = angle;
    double current = NavX.getAngle();
    double kP = 2.0;
    mecanumDrive.driveCartesian(0.0, 0.0, ((target-current)*kP)/300);
    
     }

  public double getGyro(){
    return NavX.getAngle();
  }
  public void stopDrive(){
    mecanumDrive.driveCartesian(0.0, 0.0, 0.0);
  }

  @Override
  public void periodic() {
    mecanumDrive.feed();
    // This method will be called once per scheduler run
  }
  public void initDefaultCommand(){
    setDefaultCommand(new DriveWithJoystick());
  }
}
