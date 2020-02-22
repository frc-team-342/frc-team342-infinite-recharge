/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Factory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.JetsonSubsystem;

public class ChangeColor extends CommandBase {
  /**
   * Creates a new ChangeColor.
   */

  private final JetsonSubsystem js;
  private final ControlPanelSubsystem control;

  private String startColor;
  private String previousColor;
  private String currColor;
  private int count;
  private int rotations;

  public ChangeColor() {
    // Use addRequirements() here to declare subsystem dependencies.
    control = Factory.getControl();
    js = Factory.getJetson();
    count = 0;
    rotations = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousColor = currColor = startColor = js.getColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("---------------------");
    System.out.println(js.getColor());
    System.out.println(count);
    System.out.println(rotations);
    SmartDashboard.putNumber("Rotations", rotations);
    if (count == 0) {
      if (js.getColor().equals(startColor)) {
        currColor = js.getColor();
        control.spin(0.2);
        // System.out.println("SC: " + startColor + "\tCC: " + js.getColor());
      } else if (!js.getColor().equals(startColor)) {
        count = 1;
      }
    }
    if (count == 1) {
      if (!js.getColor().equals(startColor)) {
        currColor = js.getColor();
        control.spin(0.2);
        // System.out.println("SC: " + startColor + "\tCC: " + js.getColor());
      } else if (js.getColor().equals(startColor)) {
        count = 0;
        rotations++;
      }
    }
    /*
     * currColor = js.getColor(); control.spin(0.2); System.out.println("SC: " +
     * startColor + "\tCC: " + js.getColor());
     */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    control.spin(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (rotations == 4) {
      rotations = 0;
      count = 0;
      return true;
    } else {
      return false;
    }
  }
}