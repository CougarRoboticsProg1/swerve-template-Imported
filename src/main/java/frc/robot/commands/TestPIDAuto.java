package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class TestPIDAuto extends CommandBase {

    private final SwerveSubsystem drivetrain;
    private final PIDController xController;
    private ShuffleboardTab tab = Shuffleboard.getTab("Drive Tank PID");

    private NetworkTableEntry poseOfRobot, kP, kI, kD, setPoint;
    private double pValue, iValue, dValue, setPointValue;

    public TestPIDAuto(SwerveSubsystem drivetrain, PIDController xController) {
        this.drivetrain = drivetrain;
        this.xController = xController;

        tab.add("kP", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

        tab.add("kI", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

        tab.add("kD", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        
        tab.add("Setpoint", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        
        tab.add("Pose", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    }

    @Override
    public void execute() {


        drivetrain.getPose();
    }
}
