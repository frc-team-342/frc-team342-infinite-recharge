package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Factory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAndOutake extends SubsystemBase {
  /**
   * this will control both intake and outake
   */

  private TalonSRX intake;
  private TalonSRX shooter1;
  private TalonSRX shooter2;
  private VictorSPX load1;
  private VictorSPX load2;

  private DigitalInput sensor1;
  private DigitalInput sensor2;
  private DigitalInput sensor3;

  private final double speed = 0.6;
  private final double speed2 = .75;
  private boolean isReversed = false;

  private final int current_limit = 80;
  private final int current_limit_duration = 2000;

  private double rpmsConverter = 60.0 / 1024.0;
  private double error = 250.0;
  private double hoodAngle = 50.0 * (Math.PI / 180.0);
  private double height = 77.125;

  // Gravity in in/s
  private double gravity = 386.09;

  private final LimelightSubsystem lime;

  public IntakeAndOutake() {
    intake = new TalonSRX(Constants.INTAKE_PRIMARY);
    shooter1 = new TalonSRX(Constants.LAUNCH_MOTOR_1);
    shooter2 = new TalonSRX(Constants.LAUNCH_MOTOR_2);
    load1 = new VictorSPX(Constants.INTAKE_CONVEYOR_1);
    load2 = new VictorSPX(Constants.INTAKE_CONVEYOR_2);

    sensor1 = new DigitalInput(Constants.INTAKE_SENSOR_1);
    sensor2 = new DigitalInput(Constants.INTAKE_SENSOR_2);
    sensor3 = new DigitalInput(Constants.INTAKE_SENSOR_3);

    shooter1.setInverted(true);
    shooter2.setInverted(true);
  

    shooter1.enableCurrentLimit(true);
    shooter1.configPeakCurrentLimit(current_limit);
    shooter1.configPeakCurrentDuration(current_limit_duration);

    shooter2.enableCurrentLimit(true);
    shooter2.configPeakCurrentLimit(current_limit);
    shooter2.configPeakCurrentDuration(current_limit_duration);

    shooter1.configAllowableClosedloopError(0, 0, 25);
    shooter1.selectProfileSlot(0, 0);
    shooter1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    shooter1.setSensorPhase(false);

    shooter1.config_kF(0, 0.015);
    shooter1.config_kP(0, 0.03);
    shooter1.config_kI(0, 0.0);
    shooter1.config_kD(0, 0.0);

    lime = Factory.getLimelight();

  }

  public void intake() {
    intake.set(ControlMode.PercentOutput, speed2);
    load1.set(ControlMode.PercentOutput, speed);

    if (!sensor3.get())
      load2.set(ControlMode.PercentOutput, 0.0);
    else
      load2.set(ControlMode.PercentOutput, 0.6);
  }

  public void reverseIntake(){
    intake.set(ControlMode.PercentOutput, -speed2);
    load1.set(ControlMode.PercentOutput, -speed);
  }

  public void setReversed(){
    isReversed = !isReversed;
    SmartDashboard.putBoolean("Intake Is Reversed", isReversed);
  }

  public boolean getReversed(){
    return isReversed;
  }

  public void outake() {
    double numerator = (Math.sqrt(gravity) * Math.sqrt(lime.getDistance()) * Math.sqrt(Math.pow(Math.tan(hoodAngle), 2) + 1.0));
    double denominator = Math.sqrt(2 * Math.tan(hoodAngle) - (2 * (height) / lime.getDistance()));

    double inchPerSec = 2 * (numerator / denominator);
    double unitConversion = 819.2/(6.0*Math.PI);

    double velocity = inchPerSec*unitConversion;

    shooter2.follow(shooter1);
    shooter1.set(ControlMode.Velocity, velocity);

    System.out.println("Velocity: " + shooter1.getSelectedSensorVelocity());

    if (shooter1.getSelectedSensorVelocity() + error < velocity && !sensor3.get())
      load2.set(ControlMode.PercentOutput, 0.0);
    else
      load2.set(ControlMode.PercentOutput, 0.6);
    load1.set(ControlMode.PercentOutput, 0.6);

  }

  public void outake(double velocity){
    shooter2.follow(shooter1);
    shooter1.set(ControlMode.Velocity, velocity);

    System.out.println("Velocity: " + shooter1.getSelectedSensorVelocity());

    if (shooter1.getSelectedSensorVelocity() + error < velocity && !sensor3.get()){
      load2.set(ControlMode.PercentOutput, 0.0);
      load1.set(ControlMode.PercentOutput, 0.0);
  }
    else{
      load2.set(ControlMode.PercentOutput, 0.6);
      load1.set(ControlMode.PercentOutput, 0.6);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter 1 Percent: ", shooter1.getMotorOutputPercent());
    SmartDashboard.putNumber("Shooter 1 Voltage: ", shooter1.getMotorOutputVoltage());
    SmartDashboard.putNumber("Shooter 1 Current: ", shooter1.getSupplyCurrent());

    SmartDashboard.putNumber("Shooter 2 Percent: ", shooter2.getMotorOutputPercent());
    SmartDashboard.putNumber("Shooter 2 Voltage: ", shooter2.getMotorOutputVoltage());
    SmartDashboard.putNumber("Shooter 2 Current: ", shooter2.getSupplyCurrent());

    SmartDashboard.putNumber("Velocity: ", shooter1.getSelectedSensorVelocity());
    // if (sensor1.get() && sensor2.get() && sensor3.get())
    // intakeStop();
  }

  public void getSensors() {
    SmartDashboard.putBoolean("Intake Sensor1: ", !sensor1.get());
    SmartDashboard.putBoolean("Intake Sensor2: ", !sensor2.get());
    SmartDashboard.putBoolean("Intake Sensor3: ", !sensor3.get());
  }

  public void intakeStop() {
    intake.set(ControlMode.PercentOutput, 0.0);
    load1.set(ControlMode.PercentOutput, 0.0);
    load2.set(ControlMode.PercentOutput, 0.0);
  }

  public void shooterStop() {
    // intake.set(ControlMode.PercentOutput, 0.0);
    load1.set(ControlMode.PercentOutput, 0.0);
    load2.set(ControlMode.PercentOutput, 0.0);
    shooter1.set(ControlMode.PercentOutput, 0.0);
    shooter2.set(ControlMode.PercentOutput, 0.0);
  }

  public double rpmsToCode(double rpms) {
    return rpms / rpmsConverter;
  }

  public double codeToRpms(double code) {
    return code * rpmsConverter;
  }
}