
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

import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeAndOutake;
import frc.robot.subsystems.LimelightSubsystem;

public class Factory {
    private static IntakeAndOutake intakeOutake = null; 
    private static DriveSystem driveSystem = null;
    private static LimelightSubsystem lime = null;
    private static IntakeAndOutake inAndOut = null;

    public static DriveSystem getDrive() {
        if (driveSystem == null)
            driveSystem = new DriveSystem(new CANSparkMax(Constants.motorL1, MotorType.kBrushless),
                    new CANSparkMax(Constants.motorL2, MotorType.kBrushless),
                    new CANSparkMax(Constants.motorR1, MotorType.kBrushless),
                    new CANSparkMax(Constants.motorR2, MotorType.kBrushless));
        return driveSystem;
    }

    public static LimelightSubsystem getLime() {
        if (lime == null)
            lime = new LimelightSubsystem();
        return lime;
    }

    public static IntakeAndOutake getIntakeOutake() {
        if (inAndOut == null)
            inAndOut = new IntakeAndOutake();
        return inAndOut;
    }

}
