/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

public class ControlPanelSubsystem extends SubsystemBase {
  // Creating a motor
  private static TalonSRX rotater;

  private static TalonSRX armMotor;

  private boolean armPlacement;

  // Encoder encoder = new Encoder(1, 2, false, EncodingType.k1X);

  // DigitalInput armLimitUp;
  // DigitalInput armLimitDown;

  // Encoder encoder = new Encoder(1, 2, false, EncodingType.k1X);

  /**
   * Creates a new ControlPanelSubsystem.
   */
  public ControlPanelSubsystem() {

    rotater = new TalonSRX(Constants.CP_ROTATE);

    armMotor = new TalonSRX(Constants.CP_ARM);
    armPlacement = true;

    // armLimitUp = new DigitalInput(Constants.ARM_LIMIT_UP);
    // armLimitDown = new DigitalInput(Constants.ARM_LIMIT_DOWN);
  }

  /**
   * Spin the control panel motor
   * 
   * @param speed The speed to spin the motor at, between -1 and 1.
   */
  public void spin(double speed) {
    MathUtil.clamp(speed, -1.0, 1.0);
    rotater.set(ControlMode.PercentOutput, speed);

    // Dividing pulses by 44.4 to find the revolutions
    // double rotations = (int) (encoder.getDistance() / 44.4);
    // System.out.println(rotations);
  }

  /**
   * Returns true if the forward limit switch is closed, or false if open.
   * 
   * @return The status of the front limit switch.
   */
  public boolean getForwardLimit() {
    return (armMotor.isFwdLimitSwitchClosed() == 1) ? true : false;
  }

  /**
   * Returns true if the back limit switch is closed, or false if open.
   * 
   * @return The status of the back limit switch.
   */
  public boolean getReverseLimit() {
    return (armMotor.isRevLimitSwitchClosed() == 1) ? true : false;
  }
  
  /**
   * Toggles the arm placement to true or false.
   */
  public void toggleArmPlacement() {
    armPlacement = !armPlacement;
  }

  /**
   * Gets the arm location. True will be folded in, and false will be out.
   * 
   * @return The position of the arm, with true as down and false as up.
   */
  public boolean getArmPlacement() {
    return armPlacement;
  }

  /**
   * Moves the arm if it is not hitting one of the limit switches.
   */
  public void moveArm() {
    if (armPlacement && !getReverseLimit()) {
      armMotor.set(ControlMode.PercentOutput, -1.0);
    } else if (!armPlacement && !getForwardLimit()) {
      armMotor.set(ControlMode.PercentOutput, 1.0);
    } else
      stopArm();
  }

  /**
   * Sets the arm's output to 0. Behavior will change depending on brake or coast mode.
   */
  public void stopArm() {
    armMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
