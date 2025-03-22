package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveLimits;

public class SwerveDriveWithGamepad extends Command {
  private final Swerve swerve;
  private final SwerveLimits limits;

  private final SlewRateLimiter xVelLimiter;
  private final SlewRateLimiter yVelLimiter;
  private final SlewRateLimiter angularVelLimiter;

  private Rotation2d rotationTarget = null;
  private final double rotationHoldFactor;
  private static double maxSpeedForChild = 1;
  private final boolean aimAtGamePiece;

  public SwerveDriveWithGamepad(Swerve swerve, SwerveLimits limits, boolean aimAtGamePiece) {
    this.swerve = swerve;
    this.limits = limits;
    this.xVelLimiter = new SlewRateLimiter(limits.getMaxAccelTele());
    this.yVelLimiter = new SlewRateLimiter(limits.getMaxAccelTele());
    this.angularVelLimiter = new SlewRateLimiter(limits.getMaxAngularAccelTele());
    this.rotationHoldFactor = Constants.Swerve.teleAngleHoldFactor;
    this.aimAtGamePiece = aimAtGamePiece;
    addRequirements(RobotContainer.swerve);
  }

  public SwerveDriveWithGamepad(Swerve swerve, SwerveLimits limits) {
    this(swerve, limits, false);
  }

  @Override
  public void initialize() {
    // ChassisSpeeds currentSpeeds = RobotContainer.swerve.getCurrentSpeeds();
    // Translation2d hack = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
    //     .rotateBy(RobotContainer.swerve.getPose().getRotation());
    // this.xVelLimiter.reset(hack.getX());
    // this.yVelLimiter.reset(hack.getY());
    // this.angularVelLimiter.reset(currentSpeeds.omegaRadiansPerSecond);
    this.xVelLimiter.reset(0);
    this.yVelLimiter.reset(0);
    this.angularVelLimiter.reset(0);

    rotationTarget = null;
    SmartDashboard.putNumber("Speed Dial", maxSpeedForChild);
  }

  @Override
  public void execute() {
    maxSpeedForChild = SmartDashboard.getNumber("Speed Dial", 0);
    double x = -RobotContainer.driver.getLeftY() * maxSpeedForChild;
    x = Math.copySign(x * x, x);
    double y = -RobotContainer.driver.getLeftX()* maxSpeedForChild;
    y = Math.copySign(y * y, y);
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    if (currentAlliance.isPresent() && currentAlliance.get() == Alliance.Red) {
      x = -1 * x;
      y = -1 * y;
    }
    double rot;
      rot = -RobotContainer.driver.getRightX() * maxSpeedForChild;
      rot = Math.copySign(rot * rot, rot);

    double targetAngularVel = rot * limits.getMaxAngularVelTele();
    boolean stop = x == 0 && y == 0 && rot == 0;

    // Only take over game piece aim if driver is not rotating and intake is
    // spinning (we don't have
    // a game piece in intake)

    double xVel = this.xVelLimiter.calculate(x * limits.getMaxVelTele());
    double yVel = this.yVelLimiter.calculate(y * limits.getMaxVelTele());
    double angularVel = this.angularVelLimiter.calculate(targetAngularVel);

    if (stop) {
      rotationTarget = null;

      RobotContainer.swerve.driveFieldRelative(new ChassisSpeeds(xVel, yVel, angularVel));
    } else {
        rotationTarget = null;      
    }

    RobotContainer.swerve.driveFieldRelative(new ChassisSpeeds(xVel, yVel, angularVel));

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
