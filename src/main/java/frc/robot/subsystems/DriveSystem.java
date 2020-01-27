/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

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

    
  }
  public void Drive(double ySpeed, double xSpeed, double zRotation) {
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation, NavX.getAngle());
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopDrive(){

    motorLeft1.set(0.0); 
    motorLeft2.set(0.0); 
    motorRight1.set(0.0);
    motorRight2.set(0.0); 

  }
}
