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

  private DigitalInput sensor1; //intake
  private DigitalInput sensor2; //hopper
  private DigitalInput sensor3; //shooter

  private final double speed = 0.75;
  private final double speed2 = .95;


  private final int current_limit = 80;
  private final int current_limit_duration = 2000;

  private double rpmsConverter = 60.0 / 1024.0;
  private double error = 250.0;

  private double hoodAngle = 40.0 * (Math.PI / 180.0);
  private double height = 90.0 - 21.125;
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
    intake = new TalonSRX(Constants.INTAKE_PRIMARY);
    shooter1 = new TalonSRX(Constants.LAUNCH_MOTOR_1);
    shooter2 = new TalonSRX(Constants.LAUNCH_MOTOR_2);
    load1 = new VictorSPX(Constants.INTAKE_CONVEYOR_1);
    load2 = new VictorSPX(Constants.INTAKE_CONVEYOR_2);

    sensor1 = new DigitalInput(Constants.INTAKE_SENSOR_1);
    sensor2 = new DigitalInput(Constants.INTAKE_SENSOR_2);
    sensor3 = new DigitalInput(Constants.INTAKE_SENSOR_3);

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

    // PID loop values for shooter

    shooter1.config_kF(0, 0.015);
    shooter1.config_kP(0, 0.03);
    shooter1.config_kI(0, 0.0);
    shooter1.config_kD(0, 0.0);

    lime = Factory.getLimelight();


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
      load2.set(ControlMode.PercentOutput, 0.0); 
    } else {
      load2.set(ControlMode.PercentOutput, speed);
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

    shooter2.follow(shooter1);
    shooter1.set(ControlMode.Velocity, velocity);

    System.out.println("Velocity: " + shooter1.getSelectedSensorVelocity());

    SmartDashboard.putNumber("Target Velocity", velocity);
    SmartDashboard.putNumber("Actual LL Dist", adjustedDist);
    SmartDashboard.putNumber("Distance Travelled", actualDist);


    if (Math.abs(shooter1.getSelectedSensorVelocity()) + error < velocity && !sensor3.get()){ 
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

  public void outake(double velocity){
    shooter2.follow(shooter1);

    shooter1.set(ControlMode.PercentOutput, velocity);

    System.out.println("Velocity: " + shooter1.getSelectedSensorVelocity());

    if (shooter1.getSelectedSensorVelocity() + error < velocity && !sensor3.get()){
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


    //SmartDashboard.putNumber("Shooter 2 Percent: ", shooter2.getMotorOutputPercent());
    //SmartDashboard.putNumber("Shooter 2 Voltage: ", shooter2.getMotorOutputVoltage());
    //SmartDashboard.putNumber("Shooter 2 Current: ", shooter2.getSupplyCurrent());

    //SmartDashboard.putNumber("Velocity: ", shooter1.getSelectedSensorVelocity());
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