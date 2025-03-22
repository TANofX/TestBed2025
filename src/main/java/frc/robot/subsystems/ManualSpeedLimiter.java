package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;

public class ManualSpeedLimiter extends AdvancedSubsystem {
    private final SwerveLimits swerveLimits;

    private final String sdKeyIncreaseSpeed = getName() + "/GoFaster";
    private final String sdKeyDecreaseSpeed = getName() + "/GoSlower";

    public ManualSpeedLimiter(SwerveLimits swerveLimits) {
        super("ManualSpeedLimiter");
        this.swerveLimits = swerveLimits;

        SmartDashboard.putData(sdKeyIncreaseSpeed, Commands.runOnce(() -> {
            swerveLimits.setMaxVelTele(swerveLimits.getMaxVelTele() + 0.5);
            swerveLimits.setMaxAccelTele(swerveLimits.getMaxAccelTele() + 0.5);
        }, this));
        SmartDashboard.putData(sdKeyDecreaseSpeed, Commands.runOnce(() -> {
            swerveLimits.setMaxVelTele(swerveLimits.getMaxVelTele() - 0.5);
            swerveLimits.setMaxAccelTele(swerveLimits.getMaxAccelTele() - 0.5);
        }, this));
    }

    @Override
    protected Command systemCheckCommand() {
        return Commands.none();
    }
}
