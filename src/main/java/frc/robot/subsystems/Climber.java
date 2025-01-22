// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class Climber extends AdvancedSubsystem {
  private final SparkFlex climbermotor;
  private final SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
  private final Solenoid climberpiston; 
  private final SparkClosedLoopController climbercontroller;

  /** Creates a new Climber. */
  public Climber(int motor_canid, int pcmid, int solonoidid) {
    climberpiston = new Solenoid(PneumaticsModuleType.REVPH, 0);
    climbermotor = new SparkFlex(motor_canid, MotorType.kBrushless);
    climbercontroller = climbermotor.getClosedLoopController();

    climberMotorConfig.inverted(false); // just incase :D
    climberMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    climberMotorConfig.smartCurrentLimit(100,80);
    ClosedLoopConfig climberMotorPidConfig = climberMotorConfig.closedLoop;
    climberMotorPidConfig.pid(Constants.Climber.MOTOR_KP, Constants.Climber.MOTOR_KI, Constants.Climber.MOTOR_KD);
    climberMotorConfig.smartCurrentLimit(100, 80);
    climbermotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  //HEY I ALEADY PUT IN THE STAGE GEAR RATIOS IN THE CONSTANTS!! -- Shirley C. :) -- Tanx


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climbercontroller.setReference(1, ControlType.kPosition);
    climberpiston.
  }

  @Override
  protected Command systemCheckCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'systemCheckCommand'");
  }
}
