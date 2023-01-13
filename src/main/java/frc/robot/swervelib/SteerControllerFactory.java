package frc.robot.swervelib;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.Constants;
import frc.robot.swervelib.ctre.CtreUtils;
import frc.robot.swervelib.ctre.Falcon500SteerConfiguration;
import static frc.robot.swervelib.ctre.CtreUtils.checkCtreError;

public class SteerControllerFactory {
    private final AbsoluteEncoderFactory encoderFactory;
    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

    private static final double TICKS_PER_ROTATION = 2048.0;

    // PID configuration
    private double proportionalConstant = Double.NaN;
    private double integralConstant = Double.NaN;
    private double derivativeConstant = Double.NaN;

    // Motion magic configuration
    private double velocityConstant = Double.NaN;
    private double accelerationConstant = Double.NaN;
    private double staticConstant = Double.NaN;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public SteerControllerFactory(AbsoluteEncoderFactory encoderFactory) {
        this.encoderFactory = encoderFactory;
    }

    public SteerControllerFactory withPidConstants(double proportional, double integral, double derivative) {
        this.proportionalConstant = proportional;
        this.integralConstant = integral;
        this.derivativeConstant = derivative;
        return this;
    }

    public boolean hasPidConstants() {
        return Double.isFinite(proportionalConstant) && Double.isFinite(integralConstant) && Double.isFinite(derivativeConstant);
    }

    public SteerControllerFactory withMotionMagic(double velocityConstant, double accelerationConstant, double staticConstant) {
        this.velocityConstant = velocityConstant;
        this.accelerationConstant = accelerationConstant;
        this.staticConstant = staticConstant;
        return this;
    }

    public boolean hasMotionMagic() {
        return Double.isFinite(velocityConstant) && Double.isFinite(accelerationConstant) && Double.isFinite(staticConstant);
    }

    public SteerControllerFactory withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public SteerControllerFactory withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }


    public void addDashboardEntries(ShuffleboardContainer container, SteerController controller) {
        container.addNumber("Current Angle", () -> Math.toDegrees(controller.getStateAngle()));
        container.addNumber("Target Angle", () -> Math.toDegrees(controller.getReferenceAngle()));
        container.addNumber("Absolute Encoder Angle",
                () -> Math.toDegrees(controller.absoluteEncoder.getAbsoluteAngle()));
    }

    public SteerController create(
            ShuffleboardContainer dashboardContainer,
            Falcon500SteerConfiguration steerConfiguration) {
        var controller = create(steerConfiguration);
        addDashboardEntries(dashboardContainer, controller);

        return controller;
    }

    public SteerController create(Falcon500SteerConfiguration steerConfiguration) {
        AbsoluteEncoder absoluteEncoder = encoderFactory.create(steerConfiguration.getEncoderConfiguration());

        final double sensorPositionCoefficient = 2.0 * Math.PI / TICKS_PER_ROTATION * Constants.steerReduction;
        final double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        if (hasPidConstants()) {
            motorConfiguration.slot0.kP = proportionalConstant;
            motorConfiguration.slot0.kI = integralConstant;
            motorConfiguration.slot0.kD = derivativeConstant;
        }
        if (hasMotionMagic()) {
            if (hasVoltageCompensation()) {
                motorConfiguration.slot0.kF = (1023.0 * sensorVelocityCoefficient / nominalVoltage) * velocityConstant;
            }
            // TODO: What should be done if no nominal voltage is configured? Use a default
            // voltage?

            // TODO: Make motion magic max voltages configurable or dynamically determine
            // optimal values
            motorConfiguration.motionCruiseVelocity = 2.0 / velocityConstant / sensorVelocityCoefficient;
            motorConfiguration.motionAcceleration = (8.0 - 2.0) / accelerationConstant / sensorVelocityCoefficient;
        }
        if (hasVoltageCompensation()) {
            motorConfiguration.voltageCompSaturation = nominalVoltage;
        }
        if (hasCurrentLimit()) {
            motorConfiguration.supplyCurrLimit.currentLimit = currentLimit;
            motorConfiguration.supplyCurrLimit.enable = true;
        }

        TalonFX motor = new TalonFX(steerConfiguration.getMotorPort());
        checkCtreError(motor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS),
                "Failed to configure Falcon 500 settings");

        if (hasVoltageCompensation()) {
            motor.enableVoltageCompensation(true);
        }
        checkCtreError(motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS),
                "Failed to set Falcon 500 feedback sensor");
        motor.setSensorPhase(true);
        motor.setInverted(Constants.steerInverted ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise);
        motor.setNeutralMode(NeutralMode.Brake);

        checkCtreError(motor.setSelectedSensorPosition(absoluteEncoder.getAbsoluteAngle() / sensorPositionCoefficient,
                0, CAN_TIMEOUT_MS), "Failed to set Falcon 500 encoder position");

        // Reduce CAN status frame rates
        CtreUtils.checkCtreError(
                motor.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_1_General,
                        STATUS_FRAME_GENERAL_PERIOD_MS,
                        CAN_TIMEOUT_MS),
                "Failed to configure Falcon status frame period");

        return new SteerController(motor,
                sensorPositionCoefficient,
                sensorVelocityCoefficient,
                hasMotionMagic() ? TalonFXControlMode.MotionMagic : TalonFXControlMode.Position,
                absoluteEncoder);
    }
}
