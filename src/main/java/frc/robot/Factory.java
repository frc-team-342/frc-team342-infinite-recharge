package frc.robot;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.LaunchSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Factories are being used in replace of solenoids. 
 */

public class Factory {
    private static ExampleSubsystem example = null; 
    private static IntakeSystem intake = null;
    private static LaunchSubsystem launch = null; 

    public static ExampleSubsystem getExample(){
        if (example == null){
            example = new ExampleSubsystem();
        }
        return example; 
    }
    
    public static IntakeSystem getIntake(){
        if (intake == null){
            intake = new IntakeSystem();
        }
        return intake;
    }

    public static LaunchSubsystem getLaunch(){
        if(launch == null){
            launch = new LaunchSubsystem();
        }
        return launch; 
    }



}
