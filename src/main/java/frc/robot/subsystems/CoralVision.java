package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.concurrent.TimeUnit;

public class CoralVision extends AdvancedSubsystem {

    /**
     * Name of the camera according to PhotonVision.
     * This is in NetworkTables under /photonvision/, e.g.: /photonvision/USB_camera.
     * Two separate instances of PhotonVision (e.g.: on separate Orange Pis) must not use the same camera name.
     */
    private static final String CORAL_CAMERA_NAME = "USB_camera";

    private final PhotonCamera camera;
    private final Swerve swerve;

    public CoralVision(Swerve swerve) {
        this.camera = new PhotonCamera(CORAL_CAMERA_NAME);
        this.swerve = swerve;
    }

    @SuppressWarnings("removal") // deprecated, but this is what the docs say to do
    private PhotonTrackedTarget getBestTarget() {
        var latestResult = camera.getLatestResult();
        if ( latestResult != null) {
            return latestResult.getBestTarget();
        } else {
            return null;
        }
    }

    private long lastTime = System.nanoTime();
    private Long moveStartTime = null;

    @Override
    public void periodic() {
        // driveRelative rotation positive is CCW, negative is CW
        if (System.nanoTime() - lastTime > TimeUnit.MILLISECONDS.toNanos(500)) {
            var target = getBestTarget();
            double radPerSec;
            if (target != null) {
                double yaw = target.yaw;
                System.out.printf("yaw=%s", yaw);
                // yaw is probably between -20 and 20, we want to get it to 0
                if (yaw > 5) {
                    // coral is to the right, move CW
                    radPerSec = -1 * Math.min(yaw / 20, 1);
                } else if (yaw < -5) {
                    // coral is to the left, move CCW
                    radPerSec = Math.min(yaw / 20, 1);
                } else {
                    radPerSec = 0;
                }
            } else {
                radPerSec = 0;
            }
            if (radPerSec != 0) {
                System.out.printf("rotate at %s rad/sec", radPerSec);
                moveStartTime = System.nanoTime();
                swerve.driveRobotRelative(new ChassisSpeeds(0, 0, radPerSec));
            } else {
                // only stop if we have been driving for more than a second.
                if (System.nanoTime() - lastTime > TimeUnit.SECONDS.toNanos(1)) {
                    System.out.println("stop");
                    swerve.stop();
                }
            }
            lastTime = System.nanoTime();
        }
    }

    @Override
    protected Command systemCheckCommand() {
        return Commands.none();
    }
}