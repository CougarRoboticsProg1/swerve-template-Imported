package frc.robot.swervelib;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    double angleError(double targetAngle);

    default SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
    }

    void setControllerMode(IdleMode mode);

    double getVoltage();

    void set(double driveVoltage, double steerAngle);
}
