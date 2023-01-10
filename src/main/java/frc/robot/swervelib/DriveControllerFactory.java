package frc.robot.swervelib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.Constants;
import static frc.robot.swervelib.rev.RevUtils.checkNeoError;

public class DriveControllerFactory {

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public DriveControllerFactory withVoltageCompensation() {
        this.nominalVoltage = Constants.nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public DriveControllerFactory withCurrentLimit() {
        this.currentLimit = Constants.driveCurrentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public void addDashboardEntries(
            ShuffleboardContainer container,
            DriveController controller
    ) {
        container.addNumber("Current Velocity", controller::getStateVelocity);
    }


    public DriveController create(ShuffleboardContainer container, int id) {
        CANSparkMax motor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.setInverted(Constants.driveInverted);

        // Setup voltage compensation
        if (hasVoltageCompensation()) {
            checkNeoError(motor.enableVoltageCompensation(nominalVoltage), "Failed to enable voltage compensation");
        }

        if (hasCurrentLimit()) {
            checkNeoError(motor.setSmartCurrentLimit((int) currentLimit), "Failed to set current limit for NEO");
        }

        checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100), "Failed to set periodic status frame 0 rate");
        checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20), "Failed to set periodic status frame 1 rate");
        checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20), "Failed to set periodic status frame 2 rate");
        // Setup encoder
        RelativeEncoder encoder = motor.getEncoder();
        double positionConversionFactor = Math.PI * Constants.wheelDiameter * Constants.driveReduction;
        encoder.setPositionConversionFactor(positionConversionFactor);
        encoder.setVelocityConversionFactor(positionConversionFactor / 60.0);
        DriveController controller = new DriveController(motor, encoder);
        addDashboardEntries(container, controller);
        return controller;
    }
}
