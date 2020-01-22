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
  private NetworkTableEntry ty, tx, camMode, ledMode;
  private double yOffsetAngle, xOffsetAngle;
  private int cameraMode, lightMode;

  /**
   * Creates a new LimelightSubsystem.
   */
  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    ty = table.getEntry("ty");
    tx = table.getEntry("tx");
    camMode = table.getEntry("camMode");
    ledMode = table.getEntry("ledMode");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    yOffsetAngle = ty.getDouble(0.0);
    xOffsetAngle = tx.getDouble(0.0);
    cameraMode = camMode.getNumber(0).intValue();
    lightMode = ledMode.getNumber(0).intValue();
  }

  public double getDistance() {
    return (34.5 - 20) / Math.tan(yOffsetAngle * Math.PI / 180);
  }

  public double getXOffset() {
    return getDistance() * (Math.tan(xOffsetAngle * Math.PI / 180));
  }

  public void switchCamMode() {
    camMode.setNumber((cameraMode == 0) ? 1 : 0);
    ledMode.setNumber((cameraMode == 0) ? 1: 3);
  }
}
