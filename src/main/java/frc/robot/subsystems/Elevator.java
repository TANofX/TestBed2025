// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class Elevator extends AdvancedSubsystem {
  private SparkFlex elevatorMotor;
  private SparkFlexSim elevatorSim;
  private SparkClosedLoopController elevatorController;
  private RelativeEncoder elevatorEncoder;

  // This configures a simulated elevator system including gravity in the calculations.
  // We use this simulation in the simulatePeriodic() method to update the simulated state.
  private final ElevatorSim m_elevatorSim = 
    new ElevatorSim(LinearSystemId.createElevatorSystem(DCMotor.getNeoVortex(1),
                                                        Constants.Elevator.ELEVATOR_MASS,
                                                        Constants.Elevator.METERS_PER_MOTOR_REVOLUTION / (2 * Math.PI),
                                                        Constants.Elevator.GEAR_RATIO),
                    DCMotor.getNeoVortex(1),
                    Constants.Elevator.MIN_HEIGHT_METERS,
                    Constants.Elevator.MAX_HEIGHT_METERS,
                    true,
                    Constants.Elevator.STARTING_HEIGHT_METERS);

  //Constructor for the Motor, puts motor into brake mode
  public Elevator(int elevatorCanID) {
    elevatorMotor = new SparkFlex(elevatorCanID, MotorType.kBrushless);
    SparkFlexConfig newConfig = new SparkFlexConfig();
    newConfig.idleMode(IdleMode.kBrake);

    // Configure the closed loop controller. This includes the  PIDF gains and the max motion settings.
    newConfig.closedLoop.pidf(Constants.Elevator.P, Constants.Elevator.I, Constants.Elevator.D, Constants.Elevator.FF);
    newConfig.closedLoop.maxMotion.allowedClosedLoopError(0.01);
    newConfig.closedLoop.maxMotion.maxAcceleration(Constants.Elevator.MAX_ACCELERATION);
    newConfig.closedLoop.maxMotion.maxVelocity(Constants.Elevator.MAX_VELOCITY);
    newConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    elevatorMotor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorController = elevatorMotor.getClosedLoopController();

    // Configure the motor simulation using the REV Robotics Spark Flex motor model.
    elevatorSim = new SparkFlexSim(elevatorMotor, DCMotor.getNeoVortex(1));
    elevatorSim.setPosition(Constants.Elevator.STARTING_HEIGHT_METERS / Constants.Elevator.METERS_PER_MOTOR_REVOLUTION);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Calculate the input voltage for the elevator simulation. The appliedVoltage is calculated by the SparkFlex simulation.
    // The RobotController battery voltage is simulated in the RoboRioSim class.
    double inputVoltage = elevatorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
    SmartDashboard.putNumber("Elevator Simulation/Simulated Voltage", inputVoltage);
    SmartDashboard.putNumber("Elevator Simulation/Motor Position", elevatorSim.getPosition());
    SmartDashboard.putNumber("Elevator Simulation/Height", this.getElevation());
    SmartDashboard.putNumber("Elevator Simulation/Simulated Elevator Velocity", m_elevatorSim.getVelocityMetersPerSecond());
    SmartDashboard.putNumber("Elevator Simulation/Simulated Elevator Height", m_elevatorSim.getPositionMeters());

    // Update the input voltage for the elevator simulation and update the simulation.
    m_elevatorSim.setInput(inputVoltage);
    m_elevatorSim.update(0.020);

    // Compute the motor velocity from the elevator simulation state. This will be used to update the SparkFlex motor simulation.
    double motorVelocity = (m_elevatorSim.getVelocityMetersPerSecond() / Constants.Elevator.METERS_PER_MOTOR_REVOLUTION) * 60.0;
    SmartDashboard.putNumber("Elevator Simulation/Simulated Motor Velocity", motorVelocity);
    elevatorSim.iterate(motorVelocity, RobotController.getBatteryVoltage(), 0.020);

    // Update the RoboRioSim voltage to the default battery voltage based on the elevator motor current.
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getMotorCurrent()));
  }

  //Auto-generated method stub
  @Override
  protected Command systemCheckCommand() {
    return Commands.none();
  }

  //A method to change the Height of the elevator to the required height
  public void toHeightMeters(double amount) {
    double rotations = amount / Constants.Elevator.METERS_PER_MOTOR_REVOLUTION;

    elevatorController.setReference(rotations, ControlType.kMAXMotionPositionControl);
  }

  //A method to check the Elevators current height
  public double getElevation() {
    return elevatorEncoder.getPosition() * Constants.Elevator.METERS_PER_MOTOR_REVOLUTION;
  }

  public void stop() {
      elevatorMotor.stopMotor();
  }
}
