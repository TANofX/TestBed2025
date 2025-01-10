// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.pid;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TuneSparkPIDController extends Command {
  protected SparkBase tuningController;

  protected RelativeEncoder encoder;
  protected SparkClosedLoopController pidController;

  private double kP, kI, kD, kFF, kMin, kMax, kiZ;

  protected String name;

  protected int pidSlot = 0;

  /** Creates a new TuneSparkPIDController. */
  public TuneSparkPIDController(String motorName, SparkBase sparkMotor, Subsystem motorOwner) {
    tuningController = sparkMotor;
    encoder = tuningController.getEncoder();
    pidController = tuningController.getClosedLoopController();

    name = motorName;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motorOwner);
  }

  public TuneSparkPIDController(String motorName, SparkBase sparkMotor, Subsystem motorOwner, int slot) {
    this(motorName, sparkMotor, motorOwner);

    pidSlot = slot;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP = pidController.getP(pidSlot);
    kI = pidController.getI(pidSlot);
    kD = pidController.getD(pidSlot);
    kFF = pidController.getFF(pidSlot);
    kiZ = pidController.getIZone(pidSlot);
    kMin = pidController.getOutputMin(pidSlot);
    kMax = pidController.getOutputMax(pidSlot);

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

    if (p != kP) { pidController.setP(p, pidSlot); kP = p; }
    if (i != kI) { pidController.setI(i, pidSlot); kI = i; }
    if (d != kD) { pidController.setD(d, pidSlot); kD = d; }
    if (iz != kiZ) { pidController.setIZone(iz, pidSlot); kiZ = iz; }
    if (ff != kFF) { pidController.setFF(ff, pidSlot); kFF = ff; }
    if ((max != kMax) || (min != kMin)) {
      pidController.setOutputRange(min, max, pidSlot);
      kMin = min;
      kMax = max;
    }
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
