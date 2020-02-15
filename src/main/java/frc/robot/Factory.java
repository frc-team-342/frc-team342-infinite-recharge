/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeAndOutake;
import frc.robot.subsystems.LimelightSubsystem;

public class Factory {
    private static DriveSystem driveSystem = null;
    private static LimelightSubsystem lime = null;
    private static IntakeAndOutake inAndOut = null;


    public static DriveSystem getDrive(){
        if(driveSystem == null)
            driveSystem = new DriveSystem(new CANSparkMax(Constants.motorL1, MotorType.kBrushless), new CANSparkMax(Constants.motorL2, MotorType.kBrushless),
            new CANSparkMax(Constants.motorR1, MotorType.kBrushless), new CANSparkMax(Constants.motorR2, MotorType.kBrushless));
        return driveSystem;
    }

    public static LimelightSubsystem getLime(){
        if(lime == null)
            lime = new LimelightSubsystem();
        return lime;
    }

    public static IntakeAndOutake getIntakeOutake(){
        if(inAndOut == null)
            inAndOut = new IntakeAndOutake();
        return inAndOut;
    }
}
