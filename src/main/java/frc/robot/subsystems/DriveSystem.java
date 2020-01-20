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
  public DriveSystem(CANSparkMax motor1, CANSparkMax motor2, CANSparkMax motor3, CANSparkMax motor4) {
    motorRight1 = motor1;
    motorRight2 = motor2;
    motorLeft1 = motor3;
    motorLeft2 = motor4;

    // Setting current limits for NEOS
    motorRight1.setSmartCurrentLimit(50);
    motorRight2.setSmartCurrentLimit(50);
    motorLeft1.setSmartCurrentLimit(50);
    motorLeft2.setSmartCurrentLimit(50);

    // Setting compensation for NEOS
    motorRight1.enableVoltageCompensation(12.0);
    motorRight2.enableVoltageCompensation(12.0);
    motorLeft1.enableVoltageCompensation(12.0);
    motorLeft2.enableVoltageCompensation(12.0);

    mecanumDrive = new MecanumDrive(motorLeft1, motorLeft2, motorRight1, motorRight2);

    NavX = new AHRS();
  }
  // Mecanum Drive method
  public void Drive(double ySpeed, double xSpeed, double zRotation) {
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation, NavX.getAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
