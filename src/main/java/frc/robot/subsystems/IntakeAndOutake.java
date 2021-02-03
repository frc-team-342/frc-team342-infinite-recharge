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
  private CANSparkMax shooterLeader;
  private CANSparkMax shooterFollower;
  private VictorSPX load1;
  private VictorSPX load2;

  private CANPIDController leaderController;
  private CANPIDController followerController;

  private DigitalInput sensor1; //intake
  private DigitalInput sensor2; //hopper
  private DigitalInput sensor3; //shooter
  private DigitalInput sensor4; // the new one


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
  private double error = 5.0; // allowable error for shooter in RPMs
  private double hoodAngle = 50.0 * (Math.PI / 180.0); // hood angle in radians
  private double height = 98.25 - 21.125; // height between robot and middle of target measured in inches
  private double targetDepth = 30.0; // depth from front of target to back measured in inches
  private double limeToHood = 27.0; // length from limelight to shooter hood measured in inches
  private double circumferenceOfWheel = (6.0*Math.PI); // 2*pi*r to get circumference
  private double setPoint = 0.0;
  //private double rpmScalar = 1.0322;
  //private double rpmAddition = 54.8;

  /**Gravity in inches/second*/ 
  private double gravity = 386.09; //gravity in in/s

  private final LimelightSubsystem lime;

  // measures how many cells are in 
  // returns 0 if there is nothing in basket
  // returns 1 if 
  // returns 3 if 
 
  private int powerCellCount = 0; 

  /**Constructor for the class*/
  public IntakeAndOutake() {

    configureShooter();

    intake = new TalonSRX(Constants.INTAKE_PRIMARY);
    shooter1 = new TalonSRX(Constants.LAUNCH_MOTOR_1);
    shooter2 = new TalonSRX(Constants.LAUNCH_MOTOR_2);
    load1 = new VictorSPX(Constants.INTAKE_CONVEYOR_1);
    load2 = new VictorSPX(Constants.INTAKE_CONVEYOR_2);

    sensor1 = new DigitalInput(Constants.INTAKE_SENSOR_1);
    sensor2 = new DigitalInput(Constants.INTAKE_SENSOR_2);
    sensor3 = new DigitalInput(Constants.INTAKE_SENSOR_3);
    sensor4 = new DigitalInput(Constants.INTAKE_SENSOR_4); 

    // sets shooter to turn in correct direction
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
    shooterLeader = new CANSparkMax(Constants.LAUNCH_MOTOR_1, MotorType.kBrushless);
    shooterFollower = new CANSparkMax(Constants.LAUNCH_MOTOR_2, MotorType.kBrushless);
    leaderController = shooterLeader.getPIDController();
    followerController = shooterFollower.getPIDController();

    kP = 0.0;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.0001826;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    shooterLeader.setInverted(true); // shooterFollower.setInverted(true);
    shooterLeader.setSmartCurrentLimit(current_limit); /**/ shooterFollower.setSmartCurrentLimit(current_limit);
    shooterLeader.enableVoltageCompensation(voltage_comp); /**/ shooterFollower.enableVoltageCompensation(voltage_comp);
    shooterLeader.setOpenLoopRampRate(ramp_rate); /**/ shooterFollower.setOpenLoopRampRate(ramp_rate);
    shooterFollower.follow(shooterLeader, true); // Follows shooterLeader without inversion

    leaderController.setP(kP); /**/ followerController.setP(kP);
    leaderController.setI(kI); /**/ followerController.setI(kI);
    leaderController.setD(kD); /**/ followerController.setD(kD);
    leaderController.setIZone(kIz); /**/ followerController.setIZone(kIz);
    leaderController.setFF(kFF); /**/ followerController.setFF(kFF);
    leaderController.setOutputRange(kMinOutput, kMaxOutput); 
    followerController.setOutputRange(kMinOutput, kMaxOutput);
    
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set RPM Point", setPoint);
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

    /*double adjustedDist = lime.getDistance();

    double actualDist = adjustedDist + limeToHood + targetDepth;
    double numerator = Math.pow(actualDist,2) * gravity;
    double denominator = actualDist * Math.sin(2*hoodAngle)-2*height*Math.pow(Math.cos(hoodAngle),2);

    double inchPerSec = Math.sqrt((numerator / denominator));
    double unitConversion = 60.0 / circumferenceOfWheel;
    
    // Final calculated velocity given to the shooter
    //double velocity = ((inchPerSec * unitConversion) * 2.451 + 8231.1);
    double velocity = (((inchPerSec * 2.451) + 8231.1) * unitConversion);*/

    setShooterVelocity();
    load2.set(ControlMode.PercentOutput, speed2);

    System.out.println("Shooter Velocity: " + getShooterVelocity());
    //SmartDashboard.putNumber("Target Velocity", velocity);

    /*if (Math.abs(getShooterVelocity()) + error < velocity && !sensor3.get()){ 
      // Will not shoot if fly wheel isnt up to speed. stops intake if shooter sensor sees cell
      //sensor.get() returns true if nothing is sensed. ! it to make it work
      load2.set(ControlMode.PercentOutput, 0.0);
      load1.set(ControlMode.PercentOutput, 0.0);
    }
    else{
      load2.set(ControlMode.PercentOutput, 0.9);
      load1.set(ControlMode.PercentOutput, 0.9);
    }*/

  }

  /**Another layer of abstraction for shooter outake method.
   * Sets the shooterLeader motor PID controller reference in RPMs.*/
  private void setShooterVelocity(){
    //leaderController.setReference(((lime.getDistance() * 3.4967) + 1908), ControlType.kVelocity);

    /*code below allows us to poll from SMARt dashboard*/
    leaderController.setReference(setPoint, ControlType.kVelocity);
    //followerController.setReference(velocity, ControlType.kVelocity);
  }

  /***Gets the shooter motor velocity from the encoder in RPMS*/
  private double getShooterVelocity(){
    return shooterLeader.getEncoder().getVelocity();
  }

  private void pidTuner(){
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double set = SmartDashboard.getNumber("Set RPM Point", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { leaderController.setP(p); followerController.setP(p); kP = p; }
    if((i != kI)) { leaderController.setI(i); followerController.setI(i); kI = i; }
    if((d != kD)) { leaderController.setD(d); followerController.setD(d); kD = d; }
    if((iz != kIz)) { leaderController.setIZone(iz); followerController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { leaderController.setFF(ff); followerController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      leaderController.setOutputRange(min, max);
      followerController.setOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max; 
    }
    if (set != setPoint) setPoint = set;
  }

  /***Overloaded shooter outake method that takes a parameter for testing purposes*/
  public void outake(double velocity){
    setShooterVelocity();

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
    pidTuner();
  }

  /**Displays intake and outake sensors on the SmartDashboard*/
  public void getSensors() {
    SmartDashboard.putBoolean("Intake Sensor1: ", !sensor1.get());
    SmartDashboard.putBoolean("Intake Sensor2: ", !sensor2.get());
    SmartDashboard.putBoolean("Intake Sensor3: ", !sensor3.get());
    SmartDashboard.putBoolean("Intake Sensor4: ", !sensor4.get());
  }


  /**Completely stops all intake motors except shooter loader*/
  public void intakeStop() {
    intake.set(ControlMode.PercentOutput, 0.0);
    load1.set(ControlMode.PercentOutput, 0.0);
  }

  /**Stops shooter loader and all intake motors except the wheel*/
  public void shooterStop() {
    load1.set(ControlMode.PercentOutput, 0.0);
    load2.set(ControlMode.PercentOutput, 0.0);
    shooterLeader.set(0.0); // shooterFollower.set(0.0);
  }

}