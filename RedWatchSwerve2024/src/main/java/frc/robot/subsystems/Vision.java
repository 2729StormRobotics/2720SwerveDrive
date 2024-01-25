// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Transform3d;
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
  NetworkTableEntry tz = table.getEntry("tz");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry targetpose_robotspace = table.getEntry("campose");
  private double x;
  private double z;
  private double y;
  private double area;
  private double[] transformation;
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

  public double getSkew(){
    return transformation[4];
  }

   public double getRoll(){
    return transformation[3];
  }
   public double getPitch(){
    return transformation[5];
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

public double getZ() {
    return z;
  }

  // returns relative area of the target compared with FOV
  public double getArea() {
    return area; 
  }

  public double[] targetpose_robotspace() {
    return transformation;
  }

  public double xAlign(){
    setLight(1);
    double xError = 0;
    double xPower = 0;
    xError = getSkew();
    if (Math.abs(xError) < Constants.VisionConstants.kDistanceTolerance) {
      return 0;
    }

    xPower = xPower * Constants.VisionConstants.kPX;
    xPower += Math.copySign(Constants.VisionConstants.kSDrive, xPower);
    return xPower;
  }

  public double zAlign(){
    setLight(1);
    double zError = 0;
    double zPower = 0;
    zError = getZ();
    if (Math.abs(zError) < Constants.VisionConstants.kDistanceTolerance) {
      return 0;
    }
    
    zPower = zPower * Constants.VisionConstants.kPY;
    zPower += Math.copySign(Constants.VisionConstants.kSDrive, zPower);
    return zPower;
  }

  public double rotationAlign() {
    setLight(1);
    double turnError = 0;
    double turnPower = 0;
    turnError = getX();
    if (Math.abs(turnError) < Constants.VisionConstants.kTolerance) {
      return 0;
    }
    turnPower = turnError * Constants.VisionConstants.kPTurn;
    turnPower += Math.copySign(Constants.VisionConstants.kSTurn, turnPower);
    return -turnPower;
  }

  public void switchPipeline(int pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    z = tz.getDouble(0.0);

    area = ta.getDouble(0.0);
    transformation = targetpose_robotspace.getDoubleArray(new double[6]);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightZ", z);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putString("Target", Vision.target);
    SmartDashboard.putNumber("Turn Power for Align", rotationAlign());
    SmartDashboard.putNumber("Skew", getSkew());
    
  }
}