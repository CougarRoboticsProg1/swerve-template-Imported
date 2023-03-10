// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.CSE.CougarScriptObject;
import frc.robot.CSE.CougarScriptReader;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveDrivePath;
import frc.robot.commands.TestPIDAuto;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_drivetrainSubsystem = new SwerveSubsystem();
  private final XboxController m_controller = new XboxController(0);
  private CougarScriptReader reader;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    registerAutoCommands();
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            () -> m_controller.getAButton(),
            () -> m_controller.getRightTriggerAxis(),
            () -> m_controller.getRightBumper(),
            () -> m_controller.getLeftBumper()
    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Button(m_controller:: getBButton).whenPressed(new SwerveAuto(m_drivetrainSubsystem, 6));
    // Back button zeros the gyroscope
    new Button(m_controller::getYButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::zeroGyroscope, m_drivetrainSubsystem);

    new Button(m_controller:: getAButton).whileActiveContinuous(new TestPIDAuto(m_drivetrainSubsystem));
  }

  public CougarScriptReader getCougarScriptReader() {
    return reader;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  private void registerAutoCommands() {
    reader = new CougarScriptReader((Pose2d startPose) -> {
      double feetToMeters = 0.30478512648;

      Translation2d flippedXandY = new Translation2d(
        startPose.getY() * feetToMeters, startPose.getX() * feetToMeters);

      Rotation2d theta = new Rotation2d(
        startPose.getRotation().getRadians());
      
      Pose2d transformedStartPose;

      transformedStartPose = new Pose2d(flippedXandY, theta);
      m_drivetrainSubsystem.setPose(transformedStartPose);
    });

    reader.registerCommand("SwerveDrivePath", (CougarScriptObject p) -> {
        List<Translation2d> wayPoints = p.getPointList("Waypoints");
        return new SwerveDrivePath(m_drivetrainSubsystem, 
                                    p.getAngle("StartAngle").getRadians(), 
                                    p.getAngle("EndAngle").getRadians(), 
                                    wayPoints);
    });
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // // Square the axis
    // value = Math.copySign(value * value, value);

    return value;
  }
}
