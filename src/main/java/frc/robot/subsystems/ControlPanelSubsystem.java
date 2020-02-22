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

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
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

  Encoder encoder = new Encoder(1, 2, false, EncodingType.k1X);

  public ControlPanelSubsystem() {
    rotater = new TalonSRX(Constants.CP_ROTATE);
    armMotor = new TalonSRX(Constants.CP_ARM);
    armPlacement = true;
  }

  public void spin(double speed) {
    rotater.set(ControlMode.PercentOutput, speed);

    // Dividing pulses by 44.4 to find the revolutions
    double rotations = (int) (encoder.getDistance() / 44.4);
    // System.out.println(rotations);
  }

  public void setArmBoolean() {
    armPlacement = !armPlacement;
  }

  public boolean getArmBoolean() {
    return armPlacement;
  }

  public void moveArm() {
    if (armPlacement == false) {
      armMotor.set(ControlMode.PercentOutput, -1.0);
    }
    else if(armPlacement==true){
      armMotor.set(ControlMode.PercentOutput, 1.0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
