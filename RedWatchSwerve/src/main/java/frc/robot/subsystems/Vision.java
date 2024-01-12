// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public static String target = "HIGH";
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  private double x;
  private double y;
  private double area;
  public Vision() {
    table.getEntry("pipeline").setNumber(Constants.VisionConstants.kAprilTagPipeline);
    table.getEntry("ledMode").setNumber(Constants.VisionConstants.kLightOffValue);
  }

  public void setPipeline(double pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }

  public void setLight(double value) {
    table.getEntry("ledMode").setNumber(value);
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  // returns relative area of the target compared with FOV
  public double getArea() {
    return area; 
  }

  public double rotationAlign() {
    setLight(1);
    double turnError = 0;
    double turnPower = 0;
    turnError = getX();
    if (Math.abs(turnError) < Constants.VisionConstants.kTolerance) {
      return 0;
    }
    turnPower = turnError * Constants.VisionConstants.kP;
    turnPower += Math.copySign(Constants.VisionConstants.kS, turnPower);
    return -turnPower;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putString("Target", Vision.target);
    SmartDashboard.putNumber("Turn Power for Align", rotationAlign());
  }
}