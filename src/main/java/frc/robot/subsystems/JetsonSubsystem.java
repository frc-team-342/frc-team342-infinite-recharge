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

public class JetsonSubsystem extends SubsystemBase {
  /**
   * Creates a new JetsonSubsystem.
   */
  private NetworkTable table;
  private NetworkTableEntry camera, controlPanel;
  private String cameraColor, controlPanelColor;

  public JetsonSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    camera = table.getEntry("color");
    controlPanel = table.getEntry("cpColor");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    cameraColor = camera.getString("null");
    controlPanelColor = controlPanel.getString("null");
  }

  /**
   * Returns the color being seen by the camera
   */
  public String getColor() {
    return cameraColor;
  }

}
