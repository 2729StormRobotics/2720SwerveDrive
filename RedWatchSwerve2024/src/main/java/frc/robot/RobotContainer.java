// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final double m_rotationMultiplier = 1;
  private final double m_translationMultiplier = 0.6;

  private final Joystick m_translator = new Joystick(1);
  private final Joystick m_rotator = new Joystick(2);
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    addSwerveDriveElastic();
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        // new RunCommand(
        //     () -> m_robotDrive.drive(
        //         -MathUtil.applyDeadband(m_driverController.getLeftY()*m_translationMultiplier, OIConstants.kDriveDeadband),
        //         -MathUtil.applyDeadband(m_driverController.getLeftX()*m_translationMultiplier, OIConstants.kDriveDeadband),
        //         -MathUtil.applyDeadband(m_driverController.getRightX()*m_rotationMultiplier, OIConstants.kDriveDeadband),
        //         true, true),
        //     m_robotDrive));
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_translator.getY()*m_translationMultiplier, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_translator.getX()*m_translationMultiplier, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_rotator.getX()*m_rotationMultiplier, OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }
  private void addSwerveDriveElastic(){
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
    
        builder.addDoubleProperty("Front Left Angle", () -> m_robotDrive.m_frontLeft.getPosition().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_robotDrive.m_frontLeft.m_drivingEncoder.getVelocity(), null);
    
        builder.addDoubleProperty("Front Right Angle", () -> m_robotDrive.m_frontRight.getPosition().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_robotDrive.m_frontRight.m_drivingEncoder.getVelocity(), null);
    
        builder.addDoubleProperty("Back Left Angle", () -> m_robotDrive.m_rearLeft.getPosition().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_robotDrive.m_rearLeft.m_drivingEncoder.getVelocity(), null);
    
        builder.addDoubleProperty("Back Right Angle", () -> m_robotDrive.m_rearRight.getPosition().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_robotDrive.m_rearRight.m_drivingEncoder.getVelocity(), null);
    
        builder.addDoubleProperty("Robot Angle", () -> Math.toRadians(m_robotDrive.getHeading()), null);
      }
    });
    SmartDashboard.putNumber("Front Left Voltage", m_robotDrive.m_frontLeft.m_drivingSparkMax.getBusVoltage() *m_robotDrive.m_frontLeft.m_drivingSparkMax.getAppliedOutput());
    SmartDashboard.putNumber("Front Right Voltage", m_robotDrive.m_frontRight.m_drivingSparkMax.getBusVoltage() *m_robotDrive.m_frontRight.m_drivingSparkMax.getAppliedOutput());
    SmartDashboard.putNumber("Back Left Voltage", m_robotDrive.m_rearLeft.m_drivingSparkMax.getBusVoltage() *m_robotDrive.m_rearLeft.m_drivingSparkMax.getAppliedOutput());
    SmartDashboard.putNumber("Back Right Voltage", m_robotDrive.m_rearRight.m_drivingSparkMax.getBusVoltage() *m_robotDrive.m_rearRight.m_drivingSparkMax.getAppliedOutput());
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    //locks wheels
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_rotator, Button.kA.value).whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
