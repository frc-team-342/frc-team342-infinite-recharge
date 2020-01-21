
package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSystem  extends SubsystemBase {
  /**
   * Controls the three motors in charge of the intake System. 
   */
  
  private TalonSRX intakeMaster; 
  private TalonSRX intakeFollow;
  
  private DigitalInput sensor1;
  private DigitalInput sensor2;
  private DigitalInput sensor3;
  private DigitalInput sensor4; 
  private DigitalInput sensor5; 

  private final double speed = 0.75; 

  public IntakeSystem() {
    intakeMaster = new TalonSRX(Constants.INTAKE1);
    intakeFollow = new TalonSRX(Constants.INTAKE2);

    sensor1 = new DigitalInput(Constants.INTAKESENSOR1);
    sensor2 = new DigitalInput(Constants.INTAKESENSOR2);
    sensor3 = new DigitalInput(Constants.INTAKESENSOR3);
    sensor4 = new DigitalInput(Constants.INTAKESENSOR4);
    sensor5 = new DigitalInput(Constants.INTAKESENSOR5); 
  }

  public void intake(){
    intakeFollow.follow(intakeMaster); 

    intakeMaster.set(ControlMode.PercentOutput, speed); 
  }

  @Override
  public void periodic() {
    //sensor.get() will return 1 or 0, make sure Java will accept that as "true" and "false"

    if(sensor1.get() == true && sensor2.get() == true && sensor3.get() == true && sensor4.get() == true && sensor5.get() == true)
      intakeStop(); 
    
    SmartDashboard.putBoolean("Intake Sensor1: ", sensor1.get()); 
    SmartDashboard.putBoolean("Intake Sensor2: ", sensor2.get()); 
    SmartDashboard.putBoolean("Intake Sensor3: ", sensor3.get()); 
    SmartDashboard.putBoolean("Intake Sensor4: ", sensor4.get()); 
    SmartDashboard.putBoolean("Intake Sensor5: ", sensor5.get()); 

  }

  public void intakeStop(){
    intakeMaster.set(ControlMode.PercentOutput, 0.0); 
  }
}
