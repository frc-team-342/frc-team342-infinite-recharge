
package frc.robot;

import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.LimelightSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Factories are being used in place of singletons.
 */

public class Factory {
    private static IntakeSystem intake = null;
    private static LaunchSubsystem launch = null; 
    private static DriveSystem driveSystem = null;
    private static LimelightSubsystem limelightSystem = null;
    
    public static IntakeSystem getIntake(){
        if (intake == null) {
            intake = new IntakeSystem();
        }
        return intake;
    }

    public static LaunchSubsystem getLaunch(){
        if(launch == null) {
            launch = new LaunchSubsystem();
        }
        return launch; 
    }
    
    public static DriveSystem getDrive(){
        if(driveSystem == null) {
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
