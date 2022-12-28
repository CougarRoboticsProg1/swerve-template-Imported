package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;


public class SwerveAuto extends CommandBase { 
    SwerveSubsystem swerve;
    private double startTime;
    private double runTime;


    SwerveAuto(SwerveSubsystem swerve, double runTime) {
        this.swerve = swerve;
        this.runTime = runTime;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        this.startTime = getFPGATimestamp();
        super.initialize();
    }

    @Override
    public void execute() {
        swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 4, 0, new Rotation2d(Math.PI)));
    }
    
    // Called once the command ends or is interrupted.
    @Override 
    public void end(boolean interrupted) {
      swerve.drive(new ChassisSpeeds(0, 0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return (getFPGATimestamp() - startTime >= runTime); 
    }
    
}
