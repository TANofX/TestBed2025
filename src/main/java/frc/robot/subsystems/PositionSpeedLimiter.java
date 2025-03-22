package frc.robot.subsystems;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;

public class PositionSpeedLimiter extends AdvancedSubsystem {
    private final SwerveLimits swerveLimits;
    private final PoseEstimator odometry;

    public PositionSpeedLimiter(SwerveLimits swerveLimits, PoseEstimator odometry) {
        super("PositionSpeedLimiter");
        this.swerveLimits = swerveLimits;
        this.odometry = odometry;
    }

    @Override
    public void periodic() {
        Pose2d position = odometry.getEstimatedPosition();

        // the logic to get max speeds
        double y = position.getY();
        double maxVel;
        double maxAccel;
        if (y > 0) {
            maxVel = SwerveLimits.MAX_VEL;
            maxAccel = SwerveLimits.MAX_ACCEL;
        } else {
            maxVel = 1;
            maxAccel = 1;
        }

        // set the limits
        swerveLimits.setMaxVelTele(maxVel);
        swerveLimits.setMaxAccelTele(maxAccel);
    }

    @Override
    protected Command systemCheckCommand() {
        return Commands.none();
    }

}
