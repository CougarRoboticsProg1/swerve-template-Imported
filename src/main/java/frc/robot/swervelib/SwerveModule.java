package frc.robot.swervelib;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants;
import frc.robot.swervelib.ctre.CanCoderAbsoluteConfiguration;
import frc.robot.swervelib.ctre.CanCoderFactoryBuilder;
import frc.robot.swervelib.ctre.Falcon500SteerConfiguration;

public class SwerveModule {
    private final DriveController driveController;
    private final SteerController steerController;


    public SwerveModule(ShuffleboardLayout container, int driveMotorPort,
    int steerMotorPort,
    int steerEncoderPort,
    double steerOffset, double kp, double ki, double kd) {
        this.driveController = new DriveControllerFactory()
        .withVoltageCompensation().withCurrentLimit().create(
                container,
                driveMotorPort
        );
        this.steerController = new SteerControllerFactory(new CanCoderFactoryBuilder()
        .withReadingUpdatePeriod(100)
        .build()).withVoltageCompensation(Constants.nominalVoltage)
        .withPidConstants(kp, ki, kd)
        .withCurrentLimit(Constants.steerCurrentLimit).create(
                container,
                new Falcon500SteerConfiguration(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                )
        );
    }

    public void setRampRate(double rate) {
        driveController.setRampRate(rate);
    }

    public void setControllerMode(IdleMode mode) {
        driveController.setControllerMode(mode);
    }

    public double getDriveVelocity() {
        return driveController.getStateVelocity();
    }

    public double getSteerAngle() {
        return steerController.getStateAngle();
    }

    public double getVoltage() {
        return driveController.getVoltage();
    }

    public double getAngleError(double targetValue) {
        double steerAngle = targetValue;
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        double difference = steerAngle - getSteerAngle();
        // Change the target angle so the difference is in the range [-pi, pi) instead
        // of [0, 2pi)
        if (difference >= Math.PI) {
            steerAngle -= 2.0 * Math.PI;
        } else if (difference < -Math.PI) {
            steerAngle += 2.0 * Math.PI;
        }
        return steerAngle - getSteerAngle();
    }

    

    public void set(double driveVoltage, double steerAngle) {
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        double difference = steerAngle - getSteerAngle();
        // Change the target angle so the difference is in the range [-pi, pi) instead
        // of [0, 2pi)
        if (difference >= Math.PI) {
            steerAngle -= 2.0 * Math.PI;
        } else if (difference < -Math.PI) {
            steerAngle += 2.0 * Math.PI;
        }
        difference = steerAngle - getSteerAngle(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can
        // be inverted so the total
        // movement of the module is less than 90 deg
        if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into
            // the range [0, 2pi)
            steerAngle += Math.PI;
            driveVoltage *= -1.0;
        }

        // Put the target angle back into the range [0, 2pi)
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        driveController.setReferenceVoltage(driveVoltage);
        steerController.setReferenceAngle(steerAngle);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
    }
}
