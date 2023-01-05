package frc.robot.swervelib;

import com.revrobotics.CANSparkMax.IdleMode;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getVoltage();

    void setControllerMode(IdleMode mode);

    double getStateVelocity();
}
