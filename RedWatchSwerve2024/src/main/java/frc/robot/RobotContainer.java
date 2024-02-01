// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.LEDs.Color;
import frc.robot.subsystems.LEDs.LEDSegment;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Vision m_vision = new Vision();
  private final LEDs m_lights = new LEDs();

  // Will allow to choose which auto command to run from the shuffleboard
  private final SendableChooser<Command> autoChooser;
  
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Puts auto chooser onto shuffleboard
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Named Commands
    NamedCommands.registerCommand("revToSpeed", new RevToSpeed());
    NamedCommands.registerCommand("runIntake", new RunIntake());
    NamedCommands.registerCommand("shootSpeaker", new ShootSpeaker());
    NamedCommands.registerCommand("visionAlign", new VisionAlign());
    NamedCommands.registerCommand("TestIntake", new TestIntake(m_vision));
    NamedCommands.registerCommand("TestShoot", new TestShoot(m_vision));
    NamedCommands.registerCommand("ColorPurple", new setLEDs(m_lights, LEDs.purple));
    NamedCommands.registerCommand("ColorRed", new setLEDs(m_lights, LEDs.red));
    NamedCommands.registerCommand("ColorBlue", new setLEDs(m_lights, LEDs.blue));
    NamedCommands.registerCommand("ColorWhite", new setLEDs(m_lights, LEDs.white));
    NamedCommands.registerCommand("ColorRainbow",new RunCommand(()-> LEDSegment.MainStrip.setRainbowAnimation(1)));
    // RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive))


    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // // Turning is controlled by the X axis of the right stick.
        // new RunCommand(
        //     () -> m_robotDrive.drive(
        //         -MathUtil.applyDeadband(Math.copySign(m_driverController.getLeftY()*m_driverController.getLeftY(), m_driverController.getLeftY())*0.6, OIConstants.kDriveDeadband),
        //         -MathUtil.applyDeadband(Math.copySign(m_driverController.getLeftX()*m_driverController.getLeftX(), m_driverController.getLeftX())*0.6, OIConstants.kDriveDeadband),
        //         -MathUtil.applyDeadband(Math.copySign(m_driverController.getRightX()*m_driverController.getRightX(), m_driverController.getRightX())*1, OIConstants.kDriveDeadband),
        //         true, true),
        //     m_robotDrive));

        
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY()*0.6, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX()*0.6, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX()*1, OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
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

    new JoystickButton(m_driverController, Button.kA.value).whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    new JoystickButton(m_driverController, Button.kB.value).whileTrue(new TestShoot(m_vision));
    new JoystickButton(m_driverController, Button.kY.value).whileTrue(new TestIntake(m_vision));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new PathPlannerAuto("test auto");
    // m_robotDrive.zeroHeading();
    return autoChooser.getSelected();
  }
}