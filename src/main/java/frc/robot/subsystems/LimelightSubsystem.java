/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  private NetworkTable table;
  private NetworkTableEntry ty, tx;
  private double yOffsetAngle, xOffsetAngle;

  /**
   * Creates a new LimelightSubsystem.
   */
  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    ty = table.getEntry("ty");
    tx = table.getEntry("tx");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    yOffsetAngle = ty.getDouble(0.0);
    xOffsetAngle = tx.getDouble(0.0);
  }

  public double getDistance() {
    return (98.25 - 19.528) / Math.tan(yOffsetAngle * Math.PI / 180);
  }

  public double getXOffset() {
    return getDistance() * (Math.tan(xOffsetAngle * Math.PI / 180));
  }
}
