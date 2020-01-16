
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
   * 
   * TODO: include four total sensor1s in this class
   * 
   */
  
  private TalonSRX intakeMaster; 
  private TalonSRX intakeFollow;
  
  private DigitalInput sensor1;
  private DigitalInput sensor2;
  private DigitalInput sensor3;
  private DigitalInput sensor4; 

  private final double speed = 0.75; 

  public IntakeSystem() {
    intakeMaster = new TalonSRX(Constants.INTAKE1);
    intakeFollow = new TalonSRX(Constants.INTAKE2);

    sensor1 = new DigitalInput(Constants.INTAKESENSOR1);
    sensor2 = new DigitalInput(Constants.INTAKESENSOR2);
    sensor3 = new DigitalInput(Constants.INTAKESENSOR3);
    sensor4 = new DigitalInput(Constants.INTAKESENSOR4);
  }

  public void intake(){
    intakeFollow.follow(intakeMaster); 

    intakeMaster.set(ControlMode.PercentOutput, speed); 
  }

  @Override
  public void periodic() {
    if(sensor1.get() == true && sensor2.get() == true && sensor3.get() == true && sensor3.get() == true)
      intakeStop(); 
    
    SmartDashboard.putBoolean("Intake Sensor1: ", sensor1.get()); 
    SmartDashboard.putBoolean("Intake Sensor2: ", sensor2.get()); 
    SmartDashboard.putBoolean("Intake Sensor3: ", sensor3.get()); 
    SmartDashboard.putBoolean("Intake Sensor4: ", sensor4.get()); 

  }

  public void intakeStop(){
    intakeMaster.set(ControlMode.PercentOutput, 0.0); 
  }
}
