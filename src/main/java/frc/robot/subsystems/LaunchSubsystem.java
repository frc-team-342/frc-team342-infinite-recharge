package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class LaunchSubsystem extends SubsystemBase {
  /**
   * launches the power cell
   * TODO: add a sensor
   */

   private TalonSRX load; 
   private TalonSRX loadFollow;
   private TalonSRX launch;

   private final double loadSpeed = 0.75; 


  public LaunchSubsystem() {
    load = new TalonSRX(Constants.LOAD);
    loadFollow = new TalonSRX(Constants.LOADFOLLOW);
    launch = new TalonSRX(Constants.LAUNCH); 

  }

  public void launch(){
    loadFollow.follow(load); 

    load.set(ControlMode.PercentOutput, loadSpeed);
  }

  @Override
  public void periodic() {
  }

  public void launchStop(){
    load.set(ControlMode.PercentOutput, 0.0); 
  }

  public void setLaunchSpeed(double speed){
    //speed is latter going to be determined by a PID loop
    launch.set(ControlMode.PercentOutput, speed); 
  }
}
