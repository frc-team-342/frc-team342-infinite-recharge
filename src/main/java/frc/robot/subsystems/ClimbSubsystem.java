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
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  /**
   * Creates a new ClimbSubsystem.
   */
  // Creating three motors
  private static TalonSRX teleMotor;
  private static TalonSRX winch1;
  private static TalonSRX winch2;

  private boolean enabled;
  private boolean teleActivated;

  public ClimbSubsystem() {
    teleMotor = new TalonSRX(Constants.CLIMB_TELESCOPE);
    winch1 = new TalonSRX(Constants.CLIMB_WINCH_1);
    winch2 = new TalonSRX(Constants.CLIMB_WINCH_2);

    enabled = false;
    teleActivated = false;

  }

  public void spinTeleMotor(double speed) {
    teleMotor.set(ControlMode.PercentOutput, Math.abs(speed));
  }

  public void spinWinchMotors(double speed) {
    if (enabled) {
      winch1.set(ControlMode.PercentOutput, Math.abs(speed));
      winch2.set(ControlMode.PercentOutput, Math.abs(speed));
    } else {
      winch1.set(ControlMode.PercentOutput, 0.0);
      winch2.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public boolean getEnable() {
    return enabled;
  }

  public void setEnable(boolean enable) {
    enabled = enable;
  }

  public boolean getActivated() {
    return teleActivated;
  }

  public void setActivated(boolean active) {
    teleActivated = active;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
