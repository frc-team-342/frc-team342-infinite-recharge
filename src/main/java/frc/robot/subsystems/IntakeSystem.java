
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSystem  extends SubsystemBase {
  /**
   * Controls the three motors in charge of the intake System.
   */
  
  private TalonSRX intake1; 
  private TalonSRX intake2;

  public IntakeSystem() {
    intake1 = new TalonSRX(Constants.INTAKE1);
    intake2 = new TalonSRX(Constants.INTAKE2);
  }

  public void intake(){
    intake2.follow(intake1); 

    
  }

  @Override
  public void periodic() {
    
  }
}
