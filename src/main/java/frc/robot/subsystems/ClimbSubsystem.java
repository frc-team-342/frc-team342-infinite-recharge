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

public class ClimbSubsystem extends SubsystemBase {
  /**
   * Creates a new ClimbSubsystem.
   */
  //Creating three motors
  private static TalonSRX teleMotor;
  private static TalonSRX winch1;

  private boolean enabled;
  private boolean teleActivated;

  public ClimbSubsystem() {
    teleMotor = new TalonSRX(Constants.climb_telescope);
    winch1 = new TalonSRX(Constants.climb_winch1);

    enabled = false;
    teleActivated = false;

  }

  public void spinTeleMotor(double speed) {
    teleMotor.set(ControlMode.PercentOutput, Math.abs(speed));
  }

  public void spinWinchMotors(double speed) {
    if (enabled) {
      winch1.set(ControlMode.PercentOutput, Math.abs(speed));
    }
    else {
      winch1.set(ControlMode.PercentOutput, 0);
    }
  }

  public boolean returnEnabler() {
    return enabled;
  }

  public void setEnablerTrue() {
    enabled = true;
  }

  public void setEnablerFalse() {
    enabled = false;
  }

  public boolean returnAcivated() {
    return teleActivated;
  }
  public void setActivatedTrue() {
    teleActivated = true;
  }
  public void setActivatedFalse() {
    teleActivated = false;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
