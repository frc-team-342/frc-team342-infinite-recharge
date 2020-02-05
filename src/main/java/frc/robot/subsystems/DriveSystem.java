/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpiutil.math.MathUtil;

public class DriveSystem extends SubsystemBase {
  private CANSparkMax motorRight1;
  private CANSparkMax motorRight2;
  private CANSparkMax motorLeft1;
  private CANSparkMax motorLeft2;

  public CANEncoder encoderL1;
  public CANEncoder encoderL2;
  public CANEncoder encoderR1;
  public CANEncoder encoderR2;

  private AHRS NavX;

  private MecanumDrive mecanumDrive;
  /**
   * Creates a new DriveSystem.
   */
  public DriveSystem(CANSparkMax motor1, CANSparkMax motor2, CANSparkMax motor3, CANSparkMax motor4) {
    motorRight1 = motor1;
    motorRight2 = motor2;
    motorLeft1 = motor3;
    motorLeft2 = motor4;

    encoderL1 = new CANEncoder(motorLeft1);
    encoderL2 = new CANEncoder(motorLeft2);
    encoderR1 = new CANEncoder(motorRight1);
    encoderR2 = new CANEncoder(motorRight2); 

    mecanumDrive = new MecanumDrive(motorLeft1, motorLeft2, motorRight1, motorRight2);

    NavX = new AHRS();


    setPID(motorRight1);
    setPID(motorRight2);
    setPID(motorLeft1);
    setPID(motorLeft2);

    motorLeft1.setInverted(true);
    motorLeft2.setInverted(true);

  }

  public void Drive(double ySpeed, double xSpeed, double zRotation) {
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation, NavX.getAngle());
  }

  public void DriveMecanum(double ySpeed, double xSpeed, double zRotation) {
    
    MathUtil.clamp(xSpeed, -1.0, 1.0);
    MathUtil.clamp(ySpeed, -1.0, 1.0);

    Vector2d input = new Vector2d(xSpeed, ySpeed);
    input.rotate(-NavX.getAngle());

    double[] speeds = new double[4];
    speeds[0] = input.x + input.y + zRotation;
    speeds[1] = -input.x + input.y - zRotation;
    speeds[2] = -input.x + input.y + zRotation;
    speeds[3] = input.x + input.y - zRotation;

    motorRight1.set(speeds[0]);
    motorRight2.set(speeds[1]);
    motorLeft1.set(speeds[2]);
    motorLeft2.set(speeds[3]);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

//TODO: go over these two methods with neal.
  public double getGyro(boolean backwards){
    double angle; 

    if(backwards){
      angle = (((((NavX.getAngle() + 180)) % 360) + 360) % 360);
    } else {
      angle = ((((NavX.getAngle()) % 360) + 360) % 360);
    }
    return angle;
  }

  public void resetGyro(){
    NavX.reset(); 
  }

  public void stopDrive(){

    motorLeft1.set(0.0); 
    motorLeft2.set(0.0); 
    motorRight1.set(0.0);
    motorRight2.set(0.0);
  }

  public void setPID(CANSparkMax motor) {
    CANPIDController pid = motor.getPIDController();
    pid.setP(0.0);
    pid.setI(0.0);
    pid.setD(0.0);
    pid.setFF(0.0);

    motor.setSmartCurrentLimit(50);
    motor.enableVoltageCompensation(12.0);
  }

  public void testPrint () {
    System.out.println("hello");
  }
}
