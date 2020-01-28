/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
  public DriveSystem(CANSparkMax motor4, CANSparkMax motor2, CANSparkMax motor3, CANSparkMax motor1) {
    motorRight1 = motor3;
    motorRight2 = motor1;
    motorLeft1 = motor4;
    motorLeft2 = motor2;

    motorLeft1.setInverted(false);
    motorLeft2.setInverted(false);
    motorRight1.setInverted(false);
    motorRight2.setInverted(false);

    motorLeft1.setSmartCurrentLimit(50);
    motorLeft2.setSmartCurrentLimit(50);
    motorRight1.setSmartCurrentLimit(50);
    motorRight2.setSmartCurrentLimit(50);

    motorLeft1.enableVoltageCompensation(12.0);
    motorLeft2.enableVoltageCompensation(12.0);
    motorRight1.enableVoltageCompensation(12.0);
    motorRight2.enableVoltageCompensation(12.0);



   // motorRight1.setInverted(true);
   // motorRight2.setInverted(true);

    mecanumDrive = new MecanumDrive(motorLeft1, motorLeft2, motorRight1, motorRight2);

    NavX = new AHRS();
  }
  public void Drive(double xSpeed, double ySpeed, double zRotation) {
    mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation/*, NavX.getAngle()*/);
  }

  public void PercentOut(double yAxis){
    motorLeft1.set(yAxis);
    motorLeft2.set(yAxis);
    motorRight1.set(yAxis);
    motorRight2.set(yAxis);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
