// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.robot.utils.*;
import frc.robot.swervelib.Mk4SwerveModuleHelper;
import frc.robot.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class SwerveSubsystem extends SubsystemBase {

	    /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static double MAX_VOLTAGE = 5.0;

	private final AHRS m_navx = new AHRS(SPI.Port.kMXP);
	private final SwerveDriveOdometry m_odometer = new SwerveDriveOdometry(
			Constants.m_kinematics,
			getGyroscopeRotation());

	private final SwerveModule m_frontLeftModule;
	private final SwerveModule m_frontRightModule;
	private final SwerveModule m_backLeftModule;
	private final SwerveModule m_backRightModule;

	private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	public SwerveSubsystem() {
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
		m_backRightModule = Mk4SwerveModuleHelper.createNeoFalcon500(
			tab.getLayout("Back Right Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(6, 0),
			Mk4SwerveModuleHelper.GearRatio.L2,
			BACK_RIGHT_MODULE_DRIVE_MOTOR,
			BACK_RIGHT_MODULE_STEER_MOTOR,
			BACK_RIGHT_MODULE_STEER_ENCODER,
			BACK_RIGHT_MODULE_STEER_OFFSET);

		m_backLeftModule = Mk4SwerveModuleHelper.createNeoFalcon500(
				tab.getLayout("Back Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(4, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				BACK_LEFT_MODULE_DRIVE_MOTOR,
				BACK_LEFT_MODULE_STEER_MOTOR,
				BACK_LEFT_MODULE_STEER_ENCODER,
				BACK_LEFT_MODULE_STEER_OFFSET);
		
		m_frontRightModule = Mk4SwerveModuleHelper.createNeoFalcon500(
				tab.getLayout("Front Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(2, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				FRONT_RIGHT_MODULE_DRIVE_MOTOR,
				FRONT_RIGHT_MODULE_STEER_MOTOR,
				FRONT_RIGHT_MODULE_STEER_ENCODER,
				FRONT_RIGHT_MODULE_STEER_OFFSET);

		m_frontLeftModule = Mk4SwerveModuleHelper.createNeoFalcon500(
				tab.getLayout("Front Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(0, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				FRONT_LEFT_MODULE_DRIVE_MOTOR,
				FRONT_LEFT_MODULE_STEER_MOTOR,
				FRONT_LEFT_MODULE_STEER_ENCODER,
				FRONT_LEFT_MODULE_STEER_OFFSET);

		setRobotIdleMode(IdleMode.kCoast);
	}

	public void setRobotIdleMode(IdleMode mode) {
		m_frontLeftModule.setControllerMode(mode);
		m_frontRightModule.setControllerMode(mode);
		m_backLeftModule.setControllerMode(mode);
		m_backRightModule.setControllerMode(mode);
	}

	public static void increaseVoltage() {
		MAX_VOLTAGE+=1;
		if(MAX_VOLTAGE > 5) {
			MAX_VOLTAGE = 5;
		}
	}

	public static void decreaseVoltage() {
		MAX_VOLTAGE-=1;
		if(MAX_VOLTAGE < 1) {
			MAX_VOLTAGE = 1;
		}
	}

	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction the
	 * robot is currently facing to the
	 * 'forwards' direction.
	 */
	public void zeroGyroscope() {
		m_navx.zeroYaw();
		m_navx.reset();
	}

	/**
	 * Return the position of the drivetrain.
	 *
	 * @return the position of the drivetrain in Pose2d
	 */
	public Pose2d getPose() {
		return m_odometer.getPoseMeters();
	}

	/**
	 * Set the position of the drivetrain
	 *
	 * @param pose the position of the drivetrain to be set
	 */
	public void setPose(Pose2d pose) {
		m_odometer.setPoseMeters(pose);
	}

	/**
	 * Reset the position of the drivetrain.
	 *
	 * @param pose the current position of the drivetrain
	 */
	public void resetOdometry() {
		m_odometer.resetPosition(getPose(), getGyroscopeRotation());
	}

	public Rotation2d getGyroscopeRotation() {
		if (m_navx.isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(-m_navx.getFusedHeading());
		}
		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		m_chassisSpeeds = chassisSpeeds;
	}

	public void setModuleStates(SwerveModuleState[] states) {
		for (SwerveModuleState i : states) {
			if (i.speedMetersPerSecond < 0.003) {
				m_frontLeftModule.set(0, m_frontLeftModule.getSteerAngle());
				m_frontRightModule.set(0, m_frontRightModule.getSteerAngle());
				m_backLeftModule.set(0, m_backLeftModule.getSteerAngle());
				m_backRightModule.set(0, m_backRightModule.getSteerAngle());
			}
		}

		m_frontLeftModule.set((states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE) 
			* Math.abs(Math.cos(m_frontLeftModule.angleError(states[0].angle.getRadians()))),
			states[0].angle.getRadians());
		m_frontRightModule.set((states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE) 
			* Math.abs(Math.cos(m_frontRightModule.angleError(states[1].angle.getRadians()))),
			states[1].angle.getRadians());
		m_backLeftModule.set((states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE)
			* Math.abs(Math.cos(m_backLeftModule.angleError(states[2].angle.getRadians()))),
			states[2].angle.getRadians());
		m_backRightModule.set((states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE)
			* Math.abs(Math.cos(m_backRightModule.angleError(states[3].angle.getRadians()))),
			states[3].angle.getRadians());
		
	  }

	@Override
	public void periodic() {
		// SmartDashboard.putNumber("Front Left Voltage", m_frontLeftModule.getVoltage());
		// SmartDashboard.putNumber("Front Right Voltage", m_frontRightModule.getVoltage());
		// SmartDashboard.putNumber("Back Left Voltage", m_backLeftModule.getVoltage());
		// SmartDashboard.putNumber("Back Right Voltage", m_backRightModule.getVoltage());
		m_odometer.update(getGyroscopeRotation(), m_frontLeftModule.getState(), m_frontRightModule.getState(),
				m_backLeftModule.getState(), m_backRightModule.getState());
		// SmartDashboard.putString("Robot Pose", getPose().toString());
		
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
		setModuleStates(states);
		
	}

	public void stop() {
		m_chassisSpeeds = new ChassisSpeeds();
	}
}
