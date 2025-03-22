package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;

public class SwerveLimitReporter extends AdvancedSubsystem {
    private final SwerveLimits swerveLimits;

    private final String sdKeyMaxVel = getName() + "/MaxVel";
    private final String sdKeyMaxAccel = getName() + "/MaxAccel";
    private final String sdKeyMaxAngularVel = getName() + "/MaxAngularVel";
    private final String sdKeyMaxAngularAccel = getName() + "/MaxAngularAccel";

    public SwerveLimitReporter(SwerveLimits swerveLimits) {
        super("SwerveLimitReporter");
        this.swerveLimits = swerveLimits;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(sdKeyMaxVel, swerveLimits.getMaxVelTele());
        SmartDashboard.putNumber(sdKeyMaxAccel, swerveLimits.getMaxAccelTele());
        SmartDashboard.putNumber(sdKeyMaxAngularVel, swerveLimits.getMaxAngularVelTele());
        SmartDashboard.putNumber(sdKeyMaxAngularAccel, swerveLimits.getMaxAngularAccelTele());
    }

    @Override
    protected Command systemCheckCommand() {
        return Commands.none();
    }
}
