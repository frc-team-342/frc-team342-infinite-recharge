/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Factory;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSystem;

public class DriveWithJoystick extends CommandBase {
  private final DriveSystem driveSystem;
  private final Joystick joy;
  private double X;
  private double Y;
  private double Z;
  /**
   * Creates a new DriveWithJoystick.
   */
  public DriveWithJoystick() {
    driveSystem = Factory.getDrive();
    joy = RobotContainer.getJoy();

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    X = joy.getX();
    Y = joy.getY();
    Z = joy.getZ();

    SmartDashboard.putNumber("X Axis", X);
    SmartDashboard.putNumber("Y Axis", Y);
    SmartDashboard.putNumber("Z Axis", Z);


    if(Math.abs(X) < 0.4){
      X = 0.0;
      SmartDashboard.putString("X Deadzone", "X is in deadzone!");
    }
    else  
      SmartDashboard.putString("X Deadzone", "X is not in deadzone!");

    if(Math.abs(Y) < 0.4){
      Y = 0.0;
      SmartDashboard.putString("Y Deadzone","Y is in deadzone!");
    }
    else
      SmartDashboard.putString("Y Deadzone", "Y is not in deadzone!");
    
    if(Math.abs(Z) < 0.4){
      Z = 0.0;
      SmartDashboard.putString("Z Deadzone", "Z is in deadzone!");
    }
    else
      SmartDashboard.putString("Z Deadzone", "Z is not in deadzone!");

    driveSystem.Drive(X, -Y, Z);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSystem.Drive(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
