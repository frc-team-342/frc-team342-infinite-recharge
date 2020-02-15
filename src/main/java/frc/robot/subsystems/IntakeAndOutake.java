package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAndOutake extends SubsystemBase {
  /**
   * this will control both intake and outake
   */

  private TalonSRX intake;
  private TalonSRX launch1;
  private TalonSRX launch2;
  private TalonSRX load1;
  private TalonSRX load2;

  private DigitalInput sensor1;
  private DigitalInput sensor2;
  private DigitalInput sensor3;

  private double accumError = 0.0;

  private final double speed = 0.5;

  public IntakeAndOutake() {
    intake = new TalonSRX(Constants.INTAKE);
    launch1 = new TalonSRX(Constants.LAUNCH1);
    launch2 = new TalonSRX(Constants.LAUNCH2);
    load1 = new TalonSRX(Constants.LOAD1);
    load2 = new TalonSRX(Constants.LOAD2);

    sensor1 = new DigitalInput(Constants.INTAKESENSOR1);
    sensor2 = new DigitalInput(Constants.INTAKESENSOR2);
    sensor3 = new DigitalInput(Constants.INTAKESENSOR3);
  }

  public void intake() {
    intake.set(ControlMode.PercentOutput, speed);
    load1.set(ControlMode.PercentOutput, speed);
    load2.set(ControlMode.PercentOutput, speed);
  }

  public void outake(double target) {
    double error = (target - launch1.getSelectedSensorVelocity());
    accumError += error;
    launch2.follow(launch1);

    double kP = 0.8;
    double kI = 1.0e-3;
    double P = error * kP;
    double I = accumError * kI;
    double speed = (P + I) / 200;
    launch1.set(ControlMode.Velocity, speed);

    if (sensor1.get() == true) {
      load1.set(ControlMode.PercentOutput, 0.0);
    }
  }

  @Override
  public void periodic() {
    if (sensor1.get() && sensor2.get() && sensor3.get())
      intakeStop();

    SmartDashboard.putBoolean("Intake Sensor1: ", sensor1.get());
    SmartDashboard.putBoolean("Intake Sensor2: ", sensor2.get());
    SmartDashboard.putBoolean("Intake Sensor3: ", sensor3.get());

  }

  public void intakeStop() {
    intake.set(ControlMode.PercentOutput, 0.0);
    load1.set(ControlMode.PercentOutput, 0.0);
    load2.set(ControlMode.PercentOutput, 0.0);
  }

  public void launchStop() {
    intake.set(ControlMode.PercentOutput, 0.0);
    load1.set(ControlMode.PercentOutput, 0.0);
    load2.set(ControlMode.PercentOutput, 0.0);
    launch1.set(ControlMode.PercentOutput, 0.0);
    launch2.set(ControlMode.PercentOutput, 0.0);
  }

  public void resetErrorAccum() {
    accumError = 0.0;
  }
}
