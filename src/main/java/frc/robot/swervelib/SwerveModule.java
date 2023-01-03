package frc.robot.swervelib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    default SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
    }

    void set(double driveVoltage, double steerAngle);
}
