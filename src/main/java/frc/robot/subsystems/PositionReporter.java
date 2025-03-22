package frc.robot.subsystems;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;

public class PositionReporter extends AdvancedSubsystem {
    private final PoseEstimator odometry;

    public PositionReporter(PoseEstimator odometry) {
        super("PositionReporter");
        this.odometry = odometry;
    }

    @Override
    public void periodic() {
        Pose2d position = odometry.getEstimatedPosition();
        SmartDashboard.putNumber(getName() + "/x", position.getX());
        SmartDashboard.putNumber(getName() + "/y", position.getY());
    }

    @Override
    protected Command systemCheckCommand() {
        return Commands.none();
    }

}
