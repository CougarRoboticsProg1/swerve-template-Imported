package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;


public class SwerveAuto extends CommandBase { 
    SwerveSubsystem swerve;
    private double startTime;
    private double runTime;
    private double checkpointTime;


    public SwerveAuto(SwerveSubsystem swerve, double runTime) {
        this.swerve = swerve;
        this.runTime = runTime;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        this.startTime = getFPGATimestamp();
        this.checkpointTime = startTime;
        super.initialize();
    }

    @Override
    public void execute() {
        if(getFPGATimestamp() - startTime < 1.5) {
            swerve.drive(new ChassisSpeeds(0, 50, 0));
            checkpointTime = getFPGATimestamp();
        } else {
            System.out.println("asdf");
            swerve.drive(new ChassisSpeeds(50, 0, 0));
            checkpointTime = getFPGATimestamp();
        }
        
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
