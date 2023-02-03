package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final SwerveSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final BooleanSupplier m_fieldRelativeSupplier;
    private final DoubleSupplier m_brakeSupplier;
    private final BooleanSupplier m_increaseSpeedSupplier;
    private final BooleanSupplier m_decreaseSpeedSupplier;

    private boolean isFieldRelative;

    public DefaultDriveCommand(SwerveSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               BooleanSupplier fieldRelativeSupplier, 
                               DoubleSupplier m_brakeSupplier, 
                               BooleanSupplier m_increaesSpeedSupplier,
                               BooleanSupplier m_decreaseSpeedSuppliuer) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_fieldRelativeSupplier = fieldRelativeSupplier;
        this.m_brakeSupplier = m_brakeSupplier;
        isFieldRelative = false;
        this.m_increaseSpeedSupplier = m_increaesSpeedSupplier;
        this.m_decreaseSpeedSupplier = m_decreaseSpeedSuppliuer;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Axis", m_translationYSupplier.getAsDouble());
        if(m_fieldRelativeSupplier.getAsBoolean()) {
            isFieldRelative = !isFieldRelative;
        }
        if(m_increaseSpeedSupplier.getAsBoolean()) {
            SwerveSubsystem.increaseVoltage();
        }
        if(m_decreaseSpeedSupplier.getAsBoolean()) {
            SwerveSubsystem.decreaseVoltage();
        }
        SmartDashboard.putBoolean("isFieldRelative", isFieldRelative);
            if(isFieldRelative) {
                m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_translationXSupplier.getAsDouble() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                            m_translationYSupplier.getAsDouble() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                            m_rotationSupplier.getAsDouble() * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                            m_drivetrainSubsystem.getGyroscopeRotation()
                    )
                );
            } else {
                m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                    m_translationXSupplier.getAsDouble() * Constants.MAX_VELOCITY_METERS_PER_SECOND,
                    m_translationYSupplier.getAsDouble() * Constants.MAX_VELOCITY_METERS_PER_SECOND, 
                    m_rotationSupplier.getAsDouble() * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
            }
        
        
    }
}
