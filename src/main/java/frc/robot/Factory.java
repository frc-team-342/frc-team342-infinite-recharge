
package frc.robot;

import frc.robot.subsystems.IntakeAndOutake;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.LimelightSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj2.command.Subsystem;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.JetsonSubsystem;

/**
 * Factories are being used in place of singletons.
 */



public class Factory {
    private static IntakeAndOutake intakeOutake = null;
    private static DriveSystem driveSystem = null;
    private static LimelightSubsystem limelightSystem = null;
    private static ControlPanelSubsystem control = null;
    private static JetsonSubsystem js = null;
    private static ClimbSubsystem cs = null;

    public static IntakeAndOutake getIntakeOutake() {
        if (intakeOutake == null) {
            intakeOutake = new IntakeAndOutake();
        }
        return intakeOutake;
    }

    
    public static DriveSystem getDrive(){
        if(driveSystem == null){
            driveSystem = new DriveSystem(
                new CANSparkMax(Constants.DRIVE_MOTOR_L1, MotorType.kBrushless), 
                new CANSparkMax(Constants.DRIVE_MOTOR_L2, MotorType.kBrushless),
                new CANSparkMax(Constants.DRIVE_MOTOR_R1, MotorType.kBrushless), 
                new CANSparkMax(Constants.DRIVE_MOTOR_R2, MotorType.kBrushless)
            );
            
        }
        return driveSystem;
    }

    public static LimelightSubsystem getLimelight() {
        if (limelightSystem == null) {
            limelightSystem = new LimelightSubsystem();
        }
        return limelightSystem;
    }

    public static ControlPanelSubsystem getControl() {
        if (control == null) {
            control = new ControlPanelSubsystem();
        }
        return control;
    }

    public static JetsonSubsystem getJetson() {
        if (js == null) {
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
