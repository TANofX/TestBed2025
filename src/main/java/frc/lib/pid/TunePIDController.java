// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.pid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TunePIDController extends Command {
  protected TunablePID pidController;

  private double kP, kI, kD, kFF, kMin, kMax, kiZ;

  protected String name;

  /** Creates a new TuneSparkPIDController. */
  public TunePIDController(String tunerName, TunablePID pid) {
    pidController = pid;

    name = tunerName;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP = pidController.getP();
    kI = pidController.getI();
    kD = pidController.getD();
    kFF = pidController.getFF();
    kiZ = pidController.getIZone();
    kMin = pidController.getOutputMin();
    kMax = pidController.getOutputMax();

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

    if (p != kP) { pidController.setP(p); kP = p; }
    if (i != kI) { pidController.setI(i); kI = i; }
    if (d != kD) { pidController.setD(d); kD = d; }
    if (iz != kiZ) { pidController.setIZone(iz); kiZ = iz; }
    if (ff != kFF) { pidController.setFF(ff); kFF = ff; }
    if ((max != kMax) || (min != kMin)) {
      pidController.setOutputRange(min, max);
      kMin = min;
      kMax = max;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
