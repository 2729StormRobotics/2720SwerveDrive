// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

public class TrackTag extends Command {
  private final DriveSubsystem m_DriveSubsystem;
  private final Vision m_vision;
  private final PIDController m_forwardController;
  private final PIDController m_horizontalController;
  private final PIDController m_rotationController;
  private final double m_tagHeight;
  /** Creates a new TrackTag. */
  public TrackTag(DriveSubsystem driveSubsystem, Vision vision, double tagHeight) {
    m_DriveSubsystem = driveSubsystem;
    m_vision = vision;

    m_forwardController = new PIDController(VisionConstants.forwardTrackP, 0, 0);
    m_horizontalController = new PIDController(VisionConstants.horizontalTrackP, 0, 0);
    m_rotationController = new PIDController(VisionConstants.rotationTrackP, 0, 0);

    m_forwardController.setTolerance(VisionConstants.forwardTolerance);
    m_horizontalController.setTolerance(VisionConstants.horizontalTolerance);
    m_rotationController.setTolerance(VisionConstants.rotationTolerance);

    m_tagHeight = tagHeight;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveSubsystem.drive(0, 0, 0, true, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveSubsystem.drive(
      m_forwardController.calculate(m_vision.getDistance(m_tagHeight), 0.5)/4,
      -m_horizontalController.calculate(m_vision.getSkew(), 0)/4, 
      m_rotationController.calculate(m_vision.getX()), false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_horizontalController.atSetpoint() && m_forwardController.atSetpoint() && m_rotationController.atSetpoint();
  }
}
