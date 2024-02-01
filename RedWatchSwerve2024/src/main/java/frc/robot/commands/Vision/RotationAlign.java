// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

public class RotationAlign extends Command {
  /** Creates a new RotationAllign. */
  Vision m_vision; 
  DriveSubsystem m_driveSubsystem;
  XboxController m_driverController;
  double turnError;
  double turnPower;

  public RotationAlign(DriveSubsystem driveSubsystem, Vision vision, XboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = vision; 
    m_driveSubsystem = driveSubsystem;
    m_driverController = driverController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnError = 0;
    turnPower = 0;
    m_vision.setLight(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turnError = m_vision.getX();
    turnPower = turnError * Constants.VisionConstants.kPTurn;
    turnPower += Math.copySign(Constants.VisionConstants.kSTurn, turnPower);

    m_driveSubsystem.drive(
      MathUtil.applyDeadband(m_driverController.getLeftY()/4, OIConstants.kDriveDeadband),
      MathUtil.applyDeadband(m_driverController.getLeftX()/4, OIConstants.kDriveDeadband),
      (-turnPower - MathUtil.applyDeadband(m_driverController.getRightX()/4, OIConstants.kDriveDeadband)),
      true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(turnError) < Constants.VisionConstants.kTolerance);
  }
}
