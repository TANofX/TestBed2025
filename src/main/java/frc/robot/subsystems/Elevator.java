// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class Elevator extends AdvancedSubsystem {
  private SparkFlex elevatorMotor;
  private SparkClosedLoopController elevatorController;
  private RelativeEncoder elevatorEncoder;
  //Constructor for the Motor, puts motor into brake mode
  public Elevator(int elevatorCanID) {
    elevatorMotor = new SparkFlex(elevatorCanID, MotorType.kBrushless);
    SparkFlexConfig newConfig = new SparkFlexConfig();
    newConfig.idleMode(IdleMode.kBrake);
    newConfig.closedLoop.pidf(Constants.Elevator.P, Constants.Elevator.I, Constants.Elevator.D, Constants.Elevator.FF);
    elevatorMotor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorController = elevatorMotor.getClosedLoopController();
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
  
  //Auto-generated method stub
  @Override
  protected Command systemCheckCommand() {
     
    throw new UnsupportedOperationException("Unimplemented method 'systemCheckCommand'");
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
}
