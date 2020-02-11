/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.JetsonSubsystem;

/**
 * Add your docs here.
 */
public class Factory {
    private static DriveSystem driveSystem = null;
    private static ExampleSubsystem exampleSS = null;
    private static ControlPanelSubsystem control = null;
    private static JetsonSubsystem js = null;
    private static ClimbSubsystem cs = null;


    public static DriveSystem getDrive(){

        if(driveSystem == null)
            driveSystem = new DriveSystem(new CANSparkMax(Constants.motorL1, MotorType.kBrushless), new CANSparkMax(Constants.motorL2, MotorType.kBrushless),
            new CANSparkMax(Constants.motorR1, MotorType.kBrushless), new CANSparkMax(Constants.motorR2, MotorType.kBrushless));
        return driveSystem;
    }

    public static ExampleSubsystem getExample(){
        if(exampleSS == null)
            exampleSS = new ExampleSubsystem();
        return exampleSS;
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
