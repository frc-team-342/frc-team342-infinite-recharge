
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSystem  extends SubsystemBase {
  /**
   * Controls the three motors in charge of the intake System.
   */
  
  private TalonSRX intakeMaster; 
  private TalonSRX intakeFollow;

  private final double speed = 0.75; 

  public IntakeSystem() {
    intakeMaster = new TalonSRX(Constants.INTAKE1);
    intakeFollow = new TalonSRX(Constants.INTAKE2);

  }

  public void intake(){
    intakeFollow.follow(intakeMaster); 

    intakeMaster.set(ControlMode.PercentOutput, speed); 
  }

  @Override
  public void periodic() {
    
  }
}
