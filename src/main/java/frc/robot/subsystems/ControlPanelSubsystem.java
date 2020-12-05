/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANDigitalInput.LimitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class ControlPanelSubsystem extends SubsystemBase {
  /**
   * Creates a new ControlPanelSubsystem.
   */
  // Creating a motor
  private static TalonSRX rotater;

  private static TalonSRX armMotor;

  private boolean armPlacement;

  // Encoder encoder = new Encoder(1, 2, false, EncodingType.k1X);

  
  //DigitalInput armLimitUp;
  //DigitalInput armLimitDown;


  //Encoder encoder = new Encoder(1, 2, false, EncodingType.k1X);

  public ControlPanelSubsystem() {

    rotater = new TalonSRX(Constants.CP_ROTATE);

    armMotor = new TalonSRX(Constants.CP_ARM);
    armPlacement = true;

    //armLimitUp = new DigitalInput(Constants.ARM_LIMIT_UP);
    //armLimitDown = new DigitalInput(Constants.ARM_LIMIT_DOWN);


  }

  public void spin(double speed) {
    rotater.set(ControlMode.PercentOutput, speed);

    // Dividing pulses by 44.4 to find the revolutions
    // double rotations = (int) (encoder.getDistance() / 44.4);
    // System.out.println(rotations);
  }

  public boolean getLimitFwd() {
    if (armMotor.isFwdLimitSwitchClosed() == 1)
      return true;
    else
      return false;
  }

  public boolean getLimitRev() {
    if (armMotor.isRevLimitSwitchClosed() == 1)
      return true;
    else
      return false;
  }


  public void setArmBoolean() {
    armPlacement = !armPlacement;
  }

  public boolean getArmBoolean() {
    return armPlacement;
  }

  public void moveArm() {
    if (armPlacement == true && !getLimitRev()) {
        armMotor.set(ControlMode.PercentOutput, -1.0);
    } else if (armPlacement == false && !getLimitFwd()) {
        armMotor.set(ControlMode.PercentOutput, 1.0);
    }
      else
        stopArm();
    }

  public void stopArm() {
    armMotor.set(ControlMode.PercentOutput, 0.0);
  }


  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("Forward Limit Switch", getLimitFwd());
    //SmartDashboard.putBoolean("Reverse Limit Switch", getLimitRev());

    //SmartDashboard.putBoolean("Arm Placement", getArmBoolean());
    // This method will be called once per scheduler run
  }
}
