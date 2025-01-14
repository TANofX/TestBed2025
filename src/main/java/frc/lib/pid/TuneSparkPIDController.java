// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.pid;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TuneSparkPIDController extends Command {
  protected SparkFlex tuningController;

  protected RelativeEncoder encoder;
  protected ClosedLoopConfig pidConfig;
  protected SparkFlexConfig motorConfig = new SparkFlexConfig();

  private double kP, kI, kD, kFF, kMin, kMax, kiZ;

  protected String name;

  protected ClosedLoopSlot pidSlot = ClosedLoopSlot.kSlot0;

  /** Creates a new TuneSparkPIDController. */
  public TuneSparkPIDController(String motorName, SparkFlex sparkMotor, Subsystem motorOwner) {
    tuningController = sparkMotor;
    encoder = tuningController.getEncoder();
    pidConfig = motorConfig.closedLoop;

    name = motorName;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motorOwner);
  }

  public TuneSparkPIDController(String motorName, SparkFlex sparkMotor, Subsystem motorOwner, ClosedLoopSlot slot) {
    this(motorName, sparkMotor, motorOwner);

    pidSlot = slot;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ClosedLoopConfigAccessor config = tuningController.configAccessor.closedLoop;
    kP = config.getP(pidSlot);
    kI = config.getI(pidSlot);
    kD = config.getD(pidSlot);
    kFF = config.getFF(pidSlot);
    kiZ = config.getIZone(pidSlot);
    kMin = config.getMinOutput(pidSlot);
    kMax = config.getMaxOutput(pidSlot);

    SmartDashboard.putNumber(name + " P Gain", kP);
    SmartDashboard.putNumber(name + " I Gain", kI);
    SmartDashboard.putNumber(name + " D Gain", kD);
    SmartDashboard.putNumber(name + " I Zone", kiZ);
    SmartDashboard.putNumber(name + " Feed Forward", kFF);
    SmartDashboard.putNumber(name + " Min Output", kMin);
    SmartDashboard.putNumber(name + " Max Output", kMax);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double p = SmartDashboard.getNumber(name + " P Gain", 0);
    double i = SmartDashboard.getNumber(name + " I Gain", 0);
    double d = SmartDashboard.getNumber(name + " D Gain", 0);
    double iz = SmartDashboard.getNumber(name + " I Zone", 0);
    double ff = SmartDashboard.getNumber(name + " Feed Forward", 0);
    double min = SmartDashboard.getNumber(name + " Min Output", 0);
    double max = SmartDashboard.getNumber(name + " Max Output", 0);

    if (p != kP) { pidConfig.p(p, pidSlot); kP = p; }
    if (i != kI) { pidConfig.i(i, pidSlot); kI = i; }
    if (d != kD) { pidConfig.d(d, pidSlot); kD = d; }
    if (iz != kiZ) { pidConfig.iZone(iz, pidSlot); kiZ = iz; }
    if (ff != kFF) { pidConfig.velocityFF(ff, pidSlot); kFF = ff; }
    if ((max != kMax) || (min != kMin)) {
      pidConfig.outputRange(min, max, pidSlot);
      kMin = min;
      kMax = max;
    }

    tuningController.configure(motorConfig, null, null);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tuningController.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
