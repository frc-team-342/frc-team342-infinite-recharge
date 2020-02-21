package frc.robot.subsystems;

import frc.robot.Constants;
<<<<<<< HEAD
=======
import frc.robot.Factory;
>>>>>>> a88a6ceef49c00fe8a9ba6045ef333137e6e1f93

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
<<<<<<< HEAD
=======
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
>>>>>>> a88a6ceef49c00fe8a9ba6045ef333137e6e1f93

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAndOutake extends SubsystemBase {
  /**
   * this will control both intake and outake
   */

  private TalonSRX intake;
<<<<<<< HEAD
  private TalonSRX launch1;
  private TalonSRX launch2; 
  private TalonSRX load1;
  private TalonSRX load2; 
=======
  private TalonSRX shooter1;
  private TalonSRX shooter2;
  private VictorSPX load1;
  private VictorSPX load2;
>>>>>>> a88a6ceef49c00fe8a9ba6045ef333137e6e1f93

  private DigitalInput sensor1;
  private DigitalInput sensor2;
  private DigitalInput sensor3;
<<<<<<< HEAD
  private DigitalInput sensor4;
  private DigitalInput sensor5;

  private final double speed = 0.75; 

  public IntakeAndOutake() {
    intake = new TalonSRX(Constants.INTAKE);
    launch1 = new TalonSRX(Constants.LAUNCH1);
    launch2 = new TalonSRX(Constants.LAUNCH2);
    load1 = new TalonSRX(Constants.LOAD1);
    load2 = new TalonSRX(Constants.LOAD2);
=======

  private final double speed = 0.3;
  private final double speed2 = .75;

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
    intake = new TalonSRX(Constants.INTAKE);
    shooter1 = new TalonSRX(Constants.shooter1);
    shooter2 = new TalonSRX(Constants.shooter2);
    load1 = new VictorSPX(Constants.LOAD1);
    load2 = new VictorSPX(Constants.LOAD2);
>>>>>>> a88a6ceef49c00fe8a9ba6045ef333137e6e1f93

    sensor1 = new DigitalInput(Constants.INTAKESENSOR1);
    sensor2 = new DigitalInput(Constants.INTAKESENSOR2);
    sensor3 = new DigitalInput(Constants.INTAKESENSOR3);
<<<<<<< HEAD
    sensor4 = new DigitalInput(Constants.INTAKESENSOR4);
    sensor5 = new DigitalInput(Constants.INTAKESENSOR5); 

    setPID(launch1, 0.18, 0.23, 0.0007, 0.0, true);
    setPID(launch2, 0.18, 0.23, 0.0007, 0.0, true);
  }

  public void intake(){
    intake.set(ControlMode.PercentOutput, speed); 
    load1.set(ControlMode.PercentOutput, speed);
    load2.set(ControlMode.PercentOutput, speed);
  }

  public void outake(double velocity){
    load1.set(ControlMode.PercentOutput, speed); 
    load2.set(ControlMode.PercentOutput, speed);
    launch1.set(ControlMode.Velocity, velocity); 
    launch2.set(ControlMode.Velocity, velocity);

    if(sensor1.get() == true){
      load1.set(ControlMode.PercentOutput, 0.0); 
    }
=======

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
    shooter1.setSensorPhase(true);

    shooter1.config_kF(0, 0.015);
    shooter1.config_kP(0, 0.03);
    shooter1.config_kI(0, 0.0);
    shooter1.config_kD(0, 0.0);

    lime = Factory.getLime();

  }

  public void intake() {
    intake.set(ControlMode.PercentOutput, speed2);
    load1.set(ControlMode.PercentOutput, speed);

    if (!sensor3.get())
      load2.set(ControlMode.PercentOutput, 0.0);
    else
      load2.set(ControlMode.PercentOutput, 0.6);
  }

  public void outake() {
    double numerator = -(Math.sqrt(gravity) * Math.sqrt(lime.getDistance()) * Math.sqrt(Math.pow(Math.tan(hoodAngle), 2) + 1.0));
    double denominator = Math.sqrt(2 * Math.tan(hoodAngle) * (2 * (height) / lime.getDistance()));

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

    if (shooter1.getSelectedSensorVelocity() + error < velocity && !sensor3.get())
      load2.set(ControlMode.PercentOutput, 0.0);
    else
      load2.set(ControlMode.PercentOutput, 0.6);
    load1.set(ControlMode.PercentOutput, 0.6);
>>>>>>> a88a6ceef49c00fe8a9ba6045ef333137e6e1f93
  }

  @Override
  public void periodic() {
<<<<<<< HEAD
    if(sensor1.get() == true && sensor2.get() == true && sensor3.get() == true && sensor4.get() == true && sensor5.get() == true)
      intakeStop();

    SmartDashboard.putBoolean("Intake Sensor1: ", sensor1.get()); 
    SmartDashboard.putBoolean("Intake Sensor2: ", sensor2.get()); 
    SmartDashboard.putBoolean("Intake Sensor3: ", sensor3.get()); 
    SmartDashboard.putBoolean("Intake Sensor4: ", sensor4.get()); 
    SmartDashboard.putBoolean("Intake Sensor5: ", sensor5.get());

  }

  public void intakeStop(){
    intake.set(ControlMode.PercentOutput, 0.0); 
    load1.set(ControlMode.PercentOutput, 0.0);
    load2.set(ControlMode.PercentOutput, 0.0); 
  }

  public void launchStop(){
    load1.set(ControlMode.PercentOutput, 0.0);
    load2.set(ControlMode.PercentOutput, 0.0);
    launch1.set(ControlMode.PercentOutput, 0.0);
    launch2.set(ControlMode.PercentOutput, 0.0); 
  }

  // public void setLaunchSpeed(double speed){
  //   //speed is latter going to be determined by a PID loop
  //   launch1.set(ControlMode.PercentOutput, speed);
  //   launch2.set(ControlMode.PercentOutput, speed);
  // }
  public void setPID(TalonSRX motor, double f, double p, double i, double d, boolean yeet){
    motor.configAllowableClosedloopError(0, 0, 1);
    motor.selectProfileSlot(0, 0);
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    motor.setSensorPhase(true);
    motor.setInverted(yeet);
    motor.config_kF(0, f);
    motor.config_kP(0, p);
    motor.config_kI(0, i);
    motor.config_kD(0, d);

=======
    SmartDashboard.putNumber("Shooter 1 Percent: ", shooter1.getMotorOutputPercent());
    SmartDashboard.putNumber("Shooter 1 Voltage: ", shooter1.getMotorOutputVoltage());
    SmartDashboard.putNumber("Shooter 1 Current: ", shooter1.getSupplyCurrent());

    SmartDashboard.putNumber("Shooter 2 Percent: ", shooter2.getMotorOutputPercent());
    SmartDashboard.putNumber("Shooter 2 Voltage: ", shooter2.getMotorOutputVoltage());
    SmartDashboard.putNumber("Shooter 2 Current: ", shooter2.getSupplyCurrent());

    SmartDashboard.putNumber("Velocity: ", codeToRpms(shooter1.getSelectedSensorVelocity()));
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
>>>>>>> a88a6ceef49c00fe8a9ba6045ef333137e6e1f93
  }
}
