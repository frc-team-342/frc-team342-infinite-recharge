package frc.robot;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeSystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Factories are being used in replace of solenoids. 
 */

public class Factory {
    private static ExampleSubsystem example = null; 
    private static DriveSystem driveSystem = null;
    private static IntakeSystem intake =null;

    public static ExampleSubsystem getExample(){
        if (example == null){
            example = new ExampleSubsystem();
        }
        return example; 
    }
    
    public static DriveSystem getDrive(){

        if(driveSystem == null)
            driveSystem = new DriveSystem(new CANSparkMax(Constants.motorL1, MotorType.kBrushless), new CANSparkMax(Constants.motorL2, MotorType.kBrushless),
            new CANSparkMax(Constants.motorR1, MotorType.kBrushless), new CANSparkMax(Constants.motorR2, MotorType.kBrushless));
        return driveSystem;
    }

    public static IntakeSystem getIntake(){
        if (intake == null){
            intake = new IntakeSystem();
        }
        return intake;
    }



}
