
package frc.robot;

import frc.robot.subsystems.IntakeAndOutake;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.LimelightSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.JetsonSubsystem;

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
            driveSystem = new DriveSystem(new CANSparkMax(Constants.drive_motorL1, MotorType.kBrushless),
                    new CANSparkMax(Constants.drive_motorL2, MotorType.kBrushless),
                    new CANSparkMax(Constants.drive_motorR1, MotorType.kBrushless),
                    new CANSparkMax(Constants.drive_motorR2, MotorType.kBrushless));
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
    
    public static LimelightSubsystem getLimelight() {
        if(lime == null) {
            lime = new LimelightSubsystem();
        }
        return lime;
    }
    public static ControlPanelSubsystem getControl(){
        if(control == null) {
            control = new ControlPanelSubsystem();
        }
        return control;
    }
    public static JetsonSubsystem getJetson(){
        if(js == null) {
            js = new JetsonSubsystem();
        }    
        return js;
    }
    public static ClimbSubsystem getClimb() {
        if (cs == null) {
            cs = new ClimbSubsystem();
        }
        return cs;
    }
}
