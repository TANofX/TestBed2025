package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;

public class SwerveLimits {
    private double maxVelTele = MAX_VEL;
    public double getMaxVelTele() {
        return maxVelTele;
    }
    public void setMaxVelTele(double maxVelTele) {
        this.maxVelTele = Math.max(0, Math.min(maxVelTele, MAX_VEL));
    }

    private double maxAccelTele = MAX_ACCEL;
    public double getMaxAccelTele() {
        return maxAccelTele;
    }
    public void setMaxAccelTele(double maxAccelTele) {
        this.maxAccelTele = Math.max(0, Math.min(maxAccelTele, MAX_ACCEL));
    }

    private double maxAngularVelTele = MAX_ANGULAR_VEL;
    public double getMaxAngularVelTele() {
        return maxAngularVelTele;
    }
    public void setMaxAngularVelTele(double maxAngularVelTele) {
        this.maxAngularVelTele = Math.max(0, Math.min(maxAngularVelTele, MAX_ANGULAR_VEL));
    }

    private double maxAngularAccelTele = MAX_ANGULAR_ACCEL;
    public double getMaxAngularAccelTele() {
        return maxAngularAccelTele;
    }
    public void setMaxAngularAccelTele(double maxAngularAccelTele) {
        this.maxAngularAccelTele = Math.max(0, Math.min(maxAngularAccelTele, MAX_ANGULAR_ACCEL));
    }

    public static final double MAX_VEL = 7.0;
    public static final double MAX_ACCEL = 10.0;
    public static final double MAX_ANGULAR_VEL = Units.degreesToRadians(180);
    public static final double MAX_ANGULAR_ACCEL = Units.degreesToRadians(540);
}
