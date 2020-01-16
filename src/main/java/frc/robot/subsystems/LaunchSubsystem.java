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

   private TalonSRX launchMaster; 
   private TalonSRX launchFollow1;
   private TalonSRX launchFollow2;

   private final double speed = 0.75; 

    private DigitalInput sensor; 

  public LaunchSubsystem() {
    launchMaster = new TalonSRX(Constants.LAUNCHMASTER);
    launchFollow1 = new TalonSRX(Constants.LAUNCHFOLLOW1);
    launchFollow2 = new TalonSRX(Constants.LAUNCHFOLLOW2); 

    sensor = new DigitalInput(Constants.LAUNCHSENSOR); 
  }

  public void launch(){
    launchFollow1.follow(launchMaster); 
    launchFollow2.follow(launchMaster); 

    launchMaster.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    if(sensor.get() == true){
      SmartDashboard.putBoolean("Launch Sensor: ", true);
      launchStop(); 
    } else {
      SmartDashboard.putBoolean("Launch Sensor: ", false); 
    }


  }

  public void launchStop(){
    launchMaster.set(ControlMode.PercentOutput, 0.0); 
  }
}
