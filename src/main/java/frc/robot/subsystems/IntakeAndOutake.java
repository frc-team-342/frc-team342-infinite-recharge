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
import com.revrobotics.ControlType;
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
  private CANSparkMax shooter2;
  private VictorSPX load1;
  private VictorSPX load2;

  private DigitalInput sensor1; //intake sensor
  private DigitalInput sensor2; //hopper sensor
  private DigitalInput sensor3; //shooter loader sensor

  private final double speed = 0.9; // Speed for shooter loader conveyor
  private final double speed2 = .95; // Speed for intake conveyors

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  private static final double ramp_rate = 0.2; // limit in seconds that motor is allowed reach full speed
  private static final double voltage_comp = 12.0; // amount of voltage allowed to be compensated
  private static final int current_limit = 60; // max amount of current motor can pull

  //Changed from 250 on 3/6/2020
  private double error = 10.0; // allowable error for shooter

  private double hoodAngle = 50.0 * (Math.PI / 180.0); // hood angle in radians
  private double height = 98.25 - 21.125; // height between robot and middle of target measured in inches
  private double targetDepth = 30.0; // depth from front of target to back measured in inches
  private double limeToHood = 27.0; // length from limelight to shooter hood measured in inches
  private double circumferenceOfWheel = (6.0*Math.PI);

  /**Gravity in inches/second*/ 
  private double gravity = 386.09;

  private final LimelightSubsystem lime;

  // measures how many cells are in 
  // returns 0 if there is nothing in basket
  // returns 1 if 
  // returns 3 if 
 
  private int powerCellCount = 0; 

  /**Constructor for the class*/
  public IntakeAndOutake() {
    configureShooter();

    intake = new TalonSRX(Constants.INTAKE_PRIMARY); // intake wheel infront of robot
    load1 = new VictorSPX(Constants.INTAKE_CONVEYOR_1); // first conveyor motor
    load2 = new VictorSPX(Constants.INTAKE_CONVEYOR_2); // second conveyor motor

    sensor1 = new DigitalInput(Constants.INTAKE_SENSOR_1); // intake sensor
    sensor2 = new DigitalInput(Constants.INTAKE_SENSOR_2); // hopper sensor
    sensor3 = new DigitalInput(Constants.INTAKE_SENSOR_3); // shooter loader sensor

    lime = Factory.getLimelight();
  }

  /**Sets all the configuration for the shooter motors. i.e inversion, encoders, instantiation, etc*/
  private void configureShooter(){
    shooter1 = new CANSparkMax(Constants.LAUNCH_MOTOR_1, MotorType.kBrushless);
    shooter2 = new CANSparkMax(Constants.LAUNCH_MOTOR_2, MotorType.kBrushless);

    kP = 0;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.001;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    shooter1.setInverted(true); /**/ shooter2.setInverted(true);
    shooter1.setSmartCurrentLimit(current_limit); /**/ shooter2.setSmartCurrentLimit(current_limit);
    shooter1.enableVoltageCompensation(voltage_comp); /**/ shooter2.enableVoltageCompensation(voltage_comp);
    shooter1.setOpenLoopRampRate(ramp_rate); /**/ shooter2.setOpenLoopRampRate(ramp_rate);

    shooter1.getPIDController().setP(kP); /**/ shooter2.getPIDController().setP(kP);
    shooter1.getPIDController().setI(kI); /**/ shooter2.getPIDController().setI(kI);
    shooter1.getPIDController().setD(kD); /**/ shooter2.getPIDController().setD(kD);
    shooter1.getPIDController().setIZone(kIz); /**/ shooter2.getPIDController().setIZone(kIz);
    shooter1.getPIDController().setFF(kFF); /**/ shooter2.getPIDController().setFF(kFF);
    shooter1.getPIDController().setOutputRange(kMinOutput, kMaxOutput); 
    shooter2.getPIDController().setOutputRange(kMinOutput, kMaxOutput);
  }

  /**Senses powercells in and out and keeps a running count of powercells currently in the robot*/
  private void powerCellCount(){
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

  /**Moves the intake conveyors and wheel to load powercells into robot*/
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

  /***Sets the intake to reverse in case of a stuck powercell*/
  public void reverseIntake(){
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

  /***Overloaded shooter method using limelight distance calculations*/
  public void outake() {
    powerCellCount();

    double adjustedDist = lime.getDistance();

    double actualDist = adjustedDist + limeToHood + targetDepth;
    double numerator = Math.pow(actualDist,2) * gravity;
    double denominator = actualDist * Math.sin(2*hoodAngle)-2*height*Math.pow(Math.cos(hoodAngle),2);

    double inchPerSec = Math.sqrt((numerator / denominator));
    double unitConversion = 60.0 / circumferenceOfWheel;
    
    // Final calculated velocity given to the shooter
    //double velocity = ((inchPerSec * unitConversion) * 2.451 + 8231.1);
    double velocity = (((inchPerSec * 2.451) + 8231.1) * unitConversion);

    setShooterVelocity(velocity);

    System.out.println("Shooter Velocity: " + getShooterVelocity());
    SmartDashboard.putNumber("Target Velocity", velocity);

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

  /**Another layer of abstraction for shooter outake method.
   * Sets the shooter1 motor PID controller reference in RPMs.*/
  private void setShooterVelocity(double velocity){
    shooter1.getPIDController().setReference(velocity, ControlType.kVelocity);
    shooter2.getPIDController().setReference(velocity, ControlType.kVelocity);
  }

  /***Gets the shooter motor velocity from the encoder in RPMS*/
  private double getShooterVelocity(){
    return shooter1.getEncoder().getVelocity();
  }

  /***Overloaded shooter outake method that takes a parameter for testing purposes*/
  public void outake(double velocity){
    setShooterVelocity(velocity);

    System.out.println("Velocity: " + getShooterVelocity());

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
    SmartDashboard.putNumber("Velocity: ", getShooterVelocity());
  }

  /**Displays intake and outake sensors on the SmartDashboard*/
  public void getSensors() {
    SmartDashboard.putBoolean("Intake Sensor1: ", !sensor1.get());
    SmartDashboard.putBoolean("Intake Sensor2: ", !sensor2.get());
    SmartDashboard.putBoolean("Intake Sensor3: ", !sensor3.get());
  }


  /**Completely stops all intake motors except shooter loader*/
  public void intakeStop() {
    intake.set(ControlMode.PercentOutput, 0.0);
    load1.set(ControlMode.PercentOutput, 0.0);
  }

  /**Stops shooter loader and all intake motors except the wheel*/
  public void shooterStop() {
    load2.set(ControlMode.PercentOutput, 0.0);
    shooter1.set(0.0); /**/ shooter2.set(0.0);
  }

}