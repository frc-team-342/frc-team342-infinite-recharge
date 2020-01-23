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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpiutil.math.MathUtil;

public class DriveSystem extends SubsystemBase {
  private CANSparkMax motorRight1;
  private CANSparkMax motorRight2;
  private CANSparkMax motorLeft1;
  private CANSparkMax motorLeft2;

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

    //mecanumDrive = new MecanumDrive(motorLeft1, motorLeft2, motorRight1, motorRight2);

    NavX = new AHRS();


  }

  public void Drive(double ySpeed, double xSpeed, double zRotation) {
    //mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation, NavX.getAngle());

    MathUtil.clamp(xSpeed, -1.0, 1.0);
    MathUtil.clamp(ySpeed, -1.0, 1.0);

    Vector2d input = new Vector2d(xSpeed, ySpeed);
    input.rotate(NavX.getAngle());

    double[] speeds = new double[4];
    speeds[0] = input.x + input.y + zRotation;
    speeds[1] = -input.x + input.y - zRotation;
    speeds[2] = -input.x + input.y + zRotation;
    speeds[3] = input.x + input.y - zRotation;



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPID(CANSparkMax motor) {
    CANPIDController pid = motor.getPIDController();
    pid.setP(0.0);
    pid.setI(0.001);
    pid.setD(0.0);
    pid.setFF(0.0);
  }
}
