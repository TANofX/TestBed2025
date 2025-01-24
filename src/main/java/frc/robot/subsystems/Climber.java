// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class Climber extends AdvancedSubsystem {
  private final SparkFlex climberMotor;
  private final SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
  private final Solenoid climberPiston; 
  private final SparkClosedLoopController climbercontroller;
  private final SingleJointedArmSim physicsSimulation;
  private final SparkFlexSim motorSimulation;
  /** Creates a new Climber. */
  public Climber(int motor_canid, int pcmid, int solonoidid) {
    climberPiston = new Solenoid(PneumaticsModuleType.REVPH, 0);
    climberMotor = new SparkFlex(motor_canid, MotorType.kBrushless);
    climbercontroller = climberMotor.getClosedLoopController();

    climberMotorConfig.inverted(false); // just incase :D
    climberMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    climberMotorConfig.smartCurrentLimit(100,80);
    ClosedLoopConfig climberMotorPidConfig = climberMotorConfig.closedLoop;
    climberMotorPidConfig.pid(Constants.Climber.MOTOR_KP, Constants.Climber.MOTOR_KI, Constants.Climber.MOTOR_KD);
    climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
    physicsSimulation = new SingleJointedArmSim(DCMotor.getNeoVortex(1), Constants.Climber.GEAR_RATIO, Constants.Climber.ARM_ANGULAR_MOMENTUM, Constants.Climber.LENGTH_METERS, Constants.Climber.MIN_ANGLE_RADS, Constants.Climber.MAX_ANGLE_RADS, false, 0);
    motorSimulation = new SparkFlexSim(climberMotor, DCMotor.getNeoVortex(1));
  }

  // HEY I ALEADY PUT IN THE STAGE GEAR RATIOS IN THE CONSTANTS!! -- Shirley C. :) -- Tanx


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climbercontroller.setReference(200, ControlType.kPosition);
  }

  @Override
  public void simulationPeriodic() {
    // Simulates gravity for the elevator
    physicsSimulation.setInputVoltage(climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    physicsSimulation.update(0.02);
    // Sets a variable for motor speed and sets the Simulation Motor's Velocity to it.
    double motorSpeed = ((physicsSimulation.getVelocityRadPerSec() / Constants.Climber.GEAR_RATIO) * 60) / (2 * Math.PI);
    motorSimulation.iterate(motorSpeed, RobotController.getBatteryVoltage(), 0.02);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(motorSimulation.getMotorCurrent()));
  }

  @Override
  protected Command systemCheckCommand() {
    // TODO Auto-generated method stub
    return Commands.none();

  }
}
