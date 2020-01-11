/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControlPanelSystem extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final Color red;
  private final Color green;
  private final Color blue;
  private final Color yellow;
  
  private Color detectedColor;
  private String colorStr;
  private ColorMatchResult result;

  private final ColorSensorV3 colorSensor;
  private final ColorMatch colorMatcher;
  /**
   * Creates a new ControlPanelSystem.
   */
  public ControlPanelSystem() {
    // Sensor and Matcher
    colorSensor = new ColorSensorV3(i2cPort);
    colorMatcher = new ColorMatch();

    // Colors that will be sensed by sensor
    red = ColorMatch.makeColor(0.561, 0.232, 0.114);
    green = ColorMatch.makeColor(0.197, 0.561, 0.240);
    blue = ColorMatch.makeColor(0.143, 0.427, 0.429);
    yellow = ColorMatch.makeColor(0.361, 0.524, 0.113);

    // Adding colors to matcher
    colorMatcher.addColorMatch(red);
    colorMatcher.addColorMatch(green);
    colorMatcher.addColorMatch(blue);
    colorMatcher.addColorMatch(yellow);


  }

  @Override
  public void periodic() {
    detectedColor = colorSensor.getColor();
    result = colorMatcher.matchClosestColor(detectedColor);

    // Checks the color and sets string to that color
    if(result.color == red)
      colorStr = "Red";
    else if(result.color == green)
      colorStr = "Green";
    else if(result.color == blue)
      colorStr = "Blue";
    else if(result.color == yellow)
      colorStr = "Yellow";
    else
      colorStr = "No Match";
    
    // Outputting numbers are strings for colors
    SmartDashboard.putNumber("Red: ", detectedColor.red);
    SmartDashboard.putNumber("Green: ", detectedColor.green);
    SmartDashboard.putNumber("Blue: ", detectedColor.blue);
    SmartDashboard.putNumber("Confidence: ", result.confidence);
    SmartDashboard.putString("Detected Color: ", colorStr);
    // This method will be called once per scheduler run
  }
}
