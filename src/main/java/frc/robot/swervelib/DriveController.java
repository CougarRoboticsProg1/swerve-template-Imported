package frc.robot.swervelib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;

public class DriveController {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    public DriveController(CANSparkMax motor, RelativeEncoder encoder) {
            this.motor = motor;
            this.encoder = encoder;
        }

    public void setReferenceVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void setReferenceVelocity(double velocity) {
        motor.set(velocity);

    }

    public double getVoltage() {
        return motor.getBusVoltage();
    }

    public double getStateVelocity() {
        return encoder.getVelocity();
    }

    public void setControllerMode(IdleMode mode) {
        motor.setIdleMode(mode);
    }

    public void setRampRate(double rate) {
        motor.setClosedLoopRampRate(rate);
    }
}
