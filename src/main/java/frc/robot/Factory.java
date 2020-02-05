
package frc.robot;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeAndOutake;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.LimelightSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Factories are being used in place of singletons.
 */

public class Factory {
    private static ExampleSubsystem example = null; 
    private static IntakeAndOutake intakeOutake = null; 
    private static DriveSystem driveSystem = null;
    private static LimelightSubsystem limelightSystem = null;

    public static ExampleSubsystem getExample(){
        if (example == null){
            example = new ExampleSubsystem();
        }
        return example; 
    }
    
    public static IntakeAndOutake getIntakeOutake(){
        if(intakeOutake == null){
            intakeOutake = new IntakeAndOutake(); 
        }
        return intakeOutake; 
    }
    
    public static DriveSystem getDrive(){
        if(driveSystem == null){
            driveSystem = new DriveSystem(
                new CANSparkMax(Constants.motorL1, MotorType.kBrushless), 
                new CANSparkMax(Constants.motorL2, MotorType.kBrushless),
                new CANSparkMax(Constants.motorR1, MotorType.kBrushless), 
                new CANSparkMax(Constants.motorR2, MotorType.kBrushless)
            );
        }
        return driveSystem;
    }
    
    public static LimelightSubsystem getLimelight() {
        if(limelightSystem == null) {
            limelightSystem = new LimelightSubsystem();
        }
        return limelightSystem;
    }

}
