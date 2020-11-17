package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Factory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAndOutake extends SubsystemBase {
  /**
   * this will control both intake and outake
   */

  private TalonSRX intake;
  private CANSparkMax shooter1;
  private VictorSPX load1;
  private VictorSPX load2;

  private CANEncoder shooterEncoder;

  private DigitalInput sensor1; //intake
  private DigitalInput sensor2; //hopper
  private DigitalInput sensor3; //shooter

  private final double speed = 0.9;
  private final double speed2 = .95;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  private static final double ramp_rate = 0.2;
  private static final double voltage_comp = 12.0;
  private static final int current_limit = 60;

  private double rpmsConverter = 60.0 / 1024.0;
  //Changed from 250 on 3/6/2020
  private double error = 50.0;

  private double hoodAngle = 50.0 * (Math.PI / 180.0);
  private double height = 98.25 - 21.125;
  private double targetDepth = 30.0;
  private double limeToHood = 27.0;

  // Gravity in in/s
  private double gravity = 386.09;

  private final LimelightSubsystem lime;

  // measures how many cells are in 
  // returns 0 if there is nothing in basket
  // returns 1 if 
  // returns 3 if 
 
  private int powerCellCount = 0; 

  public IntakeAndOutake() {
    configureShooter();

    intake = new TalonSRX(Constants.INTAKE_PRIMARY);
    load1 = new VictorSPX(Constants.INTAKE_CONVEYOR_1);
    load2 = new VictorSPX(Constants.INTAKE_CONVEYOR_2);

    sensor1 = new DigitalInput(Constants.INTAKE_SENSOR_1);
    sensor2 = new DigitalInput(Constants.INTAKE_SENSOR_2);
    sensor3 = new DigitalInput(Constants.INTAKE_SENSOR_3);

    lime = Factory.getLimelight();
  }

  public void configureShooter(){
    shooter1 = new CANSparkMax(Constants.LAUNCH_MOTOR_1, MotorType.kBrushless);
    shooterEncoder = new CANEncoder(shooter1);
    CANPIDController pid = shooter1.getPIDController();
    
    shooter1.setSmartCurrentLimit(current_limit);
    shooter1.enableVoltageCompensation(voltage_comp);
    shooter1.setOpenLoopRampRate(ramp_rate);
    shooter1.setInverted(true);

    kP = 5e-5;
    kI = 1e-6;
    kD = 0;
    kIz = 0;
    kFF = 0.000156;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    pid.setIZone(kIz);
    pid.setFF(kFF);
    pid.setOutputRange(kMinOutput, kMaxOutput);

    //shooter1.setInverted(true);
  }

  public void powerCellCount(){
    // counts power cells in and out so we dont get more than 5
    boolean isTriggered1 = !sensor1.get(); 
    boolean isTriggered2 = !sensor3.get(); 
    boolean holding1 = false;
    boolean holding2 = false; 

    if(!holding1){
      if(isTriggered1){
        holding1 = false;
      }
    } else {
      if(!isTriggered1){
        holding1 = false;
        powerCellCount++; 
      }
    }

    if(!holding2){
      if(isTriggered2){
        holding2 = false;
      }
    } else {
      if(!isTriggered2){
        holding2 = false;
        powerCellCount++; 
      }
    }

  SmartDashboard.putNumber("Power Cell Count: ", powerCellCount); 
    
  }

  public void intake() {
    powerCellCount(); 
    intake.set(ControlMode.PercentOutput, speed2);
    load1.set(ControlMode.PercentOutput, speed);

    // stops shooter loader if cell is sensed to prevent jamming of shooter
    if (!sensor3.get()){
      //occurs when hopper is full
      load2.set(ControlMode.PercentOutput, speed);
      //load2.set(ControlMode.PercentOutput, 0.0); 
    } else {
      //load2.set(ControlMode.PercentOutput, speed);
      load2.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public void reverseIntake(){
    // If cell gets stuck in the intake
    intake.set(ControlMode.PercentOutput, -speed2);
    load1.set(ControlMode.PercentOutput, -speed);

    boolean isTriggered = !sensor1.get(); 
    boolean holding = false;
    if(!holding){
      if(isTriggered){
        holding = false;
      }
    } else {
      if(!isTriggered){
        holding = false;
        powerCellCount--; 
      }
    }
    SmartDashboard.putNumber("Power Cell Count: ", powerCellCount); 
  }

  public void outake() {
    powerCellCount();

    // Not even going to try to document this lol. Distance calculation for velocity
    
    //Change on 3/6/2020 by Mr. Neal
    //double adjustedDist = (lime.getDistance() * 0.83) + 9.2;
    double adjustedDist = lime.getDistance();

    double actualDist = adjustedDist + limeToHood + targetDepth;
    double numerator = Math.pow(actualDist,2) * gravity;
    double denominator = actualDist * Math.sin(2*hoodAngle)-2*height*Math.pow(Math.cos(hoodAngle),2);

    double inchPerSec = Math.sqrt((numerator / denominator));
    double unitConversion = 819.2/(6.0*Math.PI);
    
    // double velocity = ((inchPerSec*(58.026) + 17434.0) + 155.8) / 0.75;
    // double velocity = (((inchPerSec) + 240.8) / 0.955 ) * unitConversion;
    
    double velocity = ((inchPerSec * unitConversion) * 2.451 + 8231.1);
    System.out.println("Velocity Calculated: " + velocity);

    setShooterVelocity(velocity);

    System.out.println("Velocity: " + getShooterVelocity());

    SmartDashboard.putNumber("Target Velocity", velocity);
    SmartDashboard.putNumber("Distance Travelled", actualDist);


    if (Math.abs(getShooterVelocity()) + error < velocity && !sensor3.get()){ 
      // Will not shoot if fly wheel isnt up to speed. stops intake if shooter sensor sees cell
      //sensor.get() returns true if nothing is sensed. ! it to make it work
      load2.set(ControlMode.PercentOutput, 0.0);
      load1.set(ControlMode.PercentOutput, 0.0);
    }
    else{
      load2.set(ControlMode.PercentOutput, 0.9);
      load1.set(ControlMode.PercentOutput, 0.9);
    }

  }

  private void setShooterVelocity(double velocity){
    shooter1.set(velocity);
  }

  private double getShooterVelocity(){
    return shooter1.getEncoder().getVelocity();
  }

  public void outake(double velocity){
    setShooterVelocity(velocity);

    System.out.println("Velocity: " + shooter1.getEncoder().getVelocity());

    if (Math.abs(getShooterVelocity()) + error < velocity && !sensor3.get()){
      load2.set(ControlMode.PercentOutput, 0.0);
      load1.set(ControlMode.PercentOutput, 0.0);
  }
    else{
      load2.set(ControlMode.PercentOutput, 0.9);
      load1.set(ControlMode.PercentOutput, 0.9);
    }

  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Shooter 1 Percent: ", shooter1.getMotorOutputPercent());
    //SmartDashboard.putNumber("Shooter 1 Voltage: ", shooter1.getMotorOutputVoltage());
    //SmartDashboard.putNumber("Shooter 1 Current: ", shooter1.getSupplyCurrent());

    SmartDashboard.putNumber("Velocity: ", shooter1.getEncoder().getVelocity());
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
    shooter1.set(0.0);
  }

  public double rpmsToCode(double rpms) {
    return rpms / rpmsConverter;
  }

  public double codeToRpms(double code) {
    return code * rpmsConverter;
  }
}