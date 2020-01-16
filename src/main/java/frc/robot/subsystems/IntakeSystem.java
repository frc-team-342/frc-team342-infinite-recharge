
package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSystem  extends SubsystemBase {
  /**
   * Controls the three motors in charge of the intake System.
   */
  
  private TalonSRX intakeMaster; 
  private TalonSRX intakeFollow;
  
  private DigitalInput sensor;

  private final double speed = 0.75; 

  public IntakeSystem() {
    intakeMaster = new TalonSRX(Constants.INTAKE1);
    intakeFollow = new TalonSRX(Constants.INTAKE2);

    sensor = new DigitalInput(0);

  }

  public void intake(){
    //intakeFollow.follow(intakeMaster); 

    intakeMaster.set(ControlMode.PercentOutput, speed); 
  }

  @Override
  public void periodic() {
    if(!sensor.get()){
      SmartDashboard.putBoolean("Sensor 1: ", true);
    } else {
      SmartDashboard.putBoolean("Sensor 1: ", false); 
    }
    
  }

  public void stop(){
    intakeMaster.set(ControlMode.PercentOutput, 0.0); 
  }
}
