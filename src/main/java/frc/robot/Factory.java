
package frc.robot;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSystem;

/**
 * Factories are being used in replace of solenoids. 
 */

public class Factory {
    private static ExampleSubsystem example = null; 
    private static IntakeSystem intake =null;

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


}
