// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class CoralHandler extends SubsystemBase {
  /** Creates a new CoralHandler. */
  //Creation of Motors, Encoders, and Limitswitch for CoralHandler Subsystem
  private final SparkFlex outtakeMotor = new SparkFlex(Constants.CoralHandler.outtakeMotorID, MotorType.kBrushless);
  //private final SparkClosedLoopController outtakeMotorController = outtakeMotor.getClosedLoopController();
  private final SparkFlex LRMotor = new SparkFlex(Constants.CoralHandler.LRMotorID, MotorType.kBrushless);
  //private final SparkClosedLoopController LRMotorController = LRMotor.getClosedLoopController();
  private final CANcoder LREncoder = new CANcoder(Constants.CoralHandler.LREncoderID);
  private final SparkFlex verticalMotor = new SparkFlex(Constants.CoralHandler.verticalMotorID, MotorType.kBrushless);
  //private final SparkClosedLoopController verticalMotorController = verticalMotor.getClosedLoopController();
  private final CANcoder verticalMotorEncoder = new CANcoder(Constants.CoralHandler.verticalMotorEncoderID);
  //private final SparkLimitSwitch coralLimitSwitch = outtakeMotor.getForwardLimitSwitch();
  //Type.kNormallyOpen?
  
  public CoralHandler() {
    //Using SparkFlexConfig, ClosedLoopConfig (also called PIDConfig), and CANcoderConfig to input the needed parameters for Coral Handler Motors
    SparkFlexConfig outtakeConfig = new SparkFlexConfig();
    outtakeConfig.inverted(false);
    outtakeConfig.idleMode(IdleMode.kBrake);
    outtakeMotor.configure(outtakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    SparkFlexConfig LRMotorConfig = new SparkFlexConfig();
    ClosedLoopConfig LRMotorPIDConfig = LRMotorConfig.closedLoop;
    CANcoderConfiguration LREncoderConfig = new CANcoderConfiguration();
    LRMotorConfig.inverted(false);
    LRMotorConfig.idleMode(IdleMode.kBrake);
    LRMotorPIDConfig.pid(Constants.CoralHandler.LRMotorP, Constants.CoralHandler.LRMotorI, Constants.CoralHandler.LRMotorD);
    LRMotorPIDConfig.velocityFF(Constants.CoralHandler.LRMotorFeedForward);
    LRMotorPIDConfig.iZone(Constants.CoralHandler.LRMotorIZone);
    LRMotorPIDConfig.maxMotion.maxAcceleration(Constants.CoralHandler.LRMotorMaxAccleration);
    LRMotorPIDConfig.maxMotion.maxVelocity(Constants.CoralHandler.LRMotorMaxVelocity);
    LRMotorPIDConfig.maxMotion.allowedClosedLoopError(Constants.CoralHandler.LRMotorClosedLoopError);
    LREncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    LREncoderConfig.MagnetSensor.MagnetOffset = Preferences.getDouble("horizontalRotationalOffset", 0);
    LRMotor.configure(LRMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    LREncoder.getConfigurator().apply(LREncoderConfig);

    SparkFlexConfig verticalMotorConfig = new SparkFlexConfig();
    ClosedLoopConfig verticalMotorPIDConfig = verticalMotorConfig.closedLoop;
    CANcoderConfiguration verticalEncoderConfig = new CANcoderConfiguration();
    verticalMotorConfig.inverted(false);
    verticalMotorConfig.idleMode(IdleMode.kBrake);
    verticalMotorPIDConfig.pid(Constants.CoralHandler.verticalMotorP, Constants.CoralHandler.verticalMotorI, Constants.CoralHandler.verticalMotorD);
    verticalMotorPIDConfig.velocityFF(Constants.CoralHandler.verticalMotorFeedForward);
    verticalMotorPIDConfig.iZone(Constants.CoralHandler.verticalMotorIZone);
    verticalMotorPIDConfig.maxMotion.maxAcceleration(Constants.CoralHandler.verticalMotorMaxAccleration);
    verticalMotorPIDConfig.maxMotion.maxVelocity(Constants.CoralHandler.verticalMotorMaxVelocity);
    verticalMotorPIDConfig.maxMotion.allowedClosedLoopError(Constants.CoralHandler.verticalMotorClosedLoopError);
    verticalMotor.configure(LRMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    verticalEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    verticalEncoderConfig.MagnetSensor.MagnetOffset = Preferences.getDouble("verticalRotationalOffset", 0);
    verticalMotorEncoder.getConfigurator().apply(verticalEncoderConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
