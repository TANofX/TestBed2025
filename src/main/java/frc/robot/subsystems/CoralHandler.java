// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

  /** Creates a new CoralHandler. */
public class CoralHandler extends AdvancedSubsystem {
  //Creation of Motors, Encoders, and Limitswitch for CoralHandler Subsystem
  private final SparkFlex outtakeMotor = new SparkFlex(Constants.CoralHandler.outtakeMotorID, MotorType.kBrushless);
  private final SparkFlex horizontalMotor = new SparkFlex(Constants.CoralHandler.horizontalMotorID, MotorType.kBrushless);
  private final SparkClosedLoopController horizontalMotorController = horizontalMotor.getClosedLoopController();
  private final CANcoder horizontalEncoder = new CANcoder(Constants.CoralHandler.horizontalEncoderID);
  private final SparkFlex verticalMotor = new SparkFlex(Constants.CoralHandler.verticalMotorID, MotorType.kBrushless);
  private final SparkClosedLoopController verticalMotorController = verticalMotor.getClosedLoopController();
  private final CANcoder verticalEncoder = new CANcoder(Constants.CoralHandler.verticalMotorEncoderID);
  private final SparkLimitSwitch coralLimitSwitch = outtakeMotor.getForwardLimitSwitch();
    //Type.kNormallyOpen???
  private final StatusSignal<Angle> horizontalRotationAbsoluteSignal;
  private final StatusSignal<Angle> verticalRotationAbsoluteSignal;
  private double verticalTarget;
  private double horizontalTarget;
  private double target;
  
  public CoralHandler() {
    registerHardware("Coral Intake/Outtake Motor", outtakeMotor);
    registerHardware("Coral Horizontal Motor", horizontalMotor);
    registerHardware("Coral Horizontal Encoder", horizontalEncoder);
    registerHardware("Coral Vertical Motor", verticalMotor);
    registerHardware("Coral Vertical Encoder", verticalEncoder);

    //Using SparkFlexConfig, ClosedLoopConfig (also called PIDConfig), and CANcoderConfig to input the needed parameters for Coral Handler Motors and Encoders
    SparkFlexConfig outtakeConfig = new SparkFlexConfig();
    outtakeConfig.inverted(false);
    outtakeConfig.idleMode(IdleMode.kBrake);
    outtakeMotor.configure(outtakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    SparkFlexConfig horizontalMotorConfig = new SparkFlexConfig();
    ClosedLoopConfig horizontalMotorPIDConfig = horizontalMotorConfig.closedLoop;
    CANcoderConfiguration horizontalEncoderConfig = new CANcoderConfiguration();
    horizontalMotorConfig.inverted(false);
    horizontalMotorConfig.idleMode(IdleMode.kBrake);
    horizontalMotorPIDConfig.pid(Constants.CoralHandler.horizontalMotorP, Constants.CoralHandler.horizontalMotorI, Constants.CoralHandler.horizontalMotorD);
    horizontalMotorPIDConfig.velocityFF(Constants.CoralHandler.horizontalMotorFeedForward);
    horizontalMotorPIDConfig.iZone(Constants.CoralHandler.horizontalMotorIZone);
    horizontalMotorPIDConfig.maxMotion.maxAcceleration(Constants.CoralHandler.horizontalMotorMaxAccleration);
    horizontalMotorPIDConfig.maxMotion.maxVelocity(Constants.CoralHandler.horizontalMotorMaxVelocity);
    horizontalMotorPIDConfig.maxMotion.allowedClosedLoopError(Constants.CoralHandler.horizontalMotorClosedLoopError);
    horizontalEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    horizontalEncoderConfig.MagnetSensor.MagnetOffset = Preferences.getDouble("horizontalRotationalOffset", 0);
    horizontalMotor.configure(horizontalMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    horizontalEncoder.getConfigurator().apply(horizontalEncoderConfig);
    horizontalRotationAbsoluteSignal = horizontalEncoder.getAbsolutePosition();
    horizontalRotationAbsoluteSignal.refresh();

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
    verticalMotor.configure(verticalMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    verticalEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    verticalEncoderConfig.MagnetSensor.MagnetOffset = Preferences.getDouble("verticalRotationalOffset", 0);
    verticalEncoder.getConfigurator().apply(verticalEncoderConfig);
    verticalRotationAbsoluteSignal = verticalEncoder.getAbsolutePosition();
    verticalRotationAbsoluteSignal.refresh();
  }
//methods for stopping specific motors
  public void stopOuttakeMotor() {
    outtakeMotor.stopMotor();
  }

  public void stopHorizontalMotor() {
    horizontalMotor.stopMotor();
  }

  public void stopVerticalMotor() {
    verticalMotor.stopMotor();
  }
  
  public void stopMotors() {
    outtakeMotor.stopMotor();
    horizontalMotor.stopMotor();
    verticalMotor.stopMotor();
  }
//intake/outtake motor methods
  public boolean hasCoral() {
    return coralLimitSwitch.isPressed();
  }

  public void runOuttakeMotor(double outtakeMotorSpeed) {
    outtakeMotor.set(outtakeMotorSpeed);
  }

  public void runIntakeMotor(double intakeMotorSpeed) {
    outtakeMotor.set(intakeMotorSpeed);
  }
//encoder methods for CoralHandler Position
  public double getVerticalAngle() {
    return verticalRotationAbsoluteSignal.getValueAsDouble() * 180;
    // get as angle or as double??? refresh? 
  }
  public double getHorizontalAngle() {
    return horizontalRotationAbsoluteSignal.getValueAsDouble() * 180;
    // get as angle or as double??? refresh? 
  }

  //How to make it one method
  // public void setVerticalPosition(Rotation2d targetVerticalPosition) {
  //   verticalTarget = targetVerticalPosition.getDegrees();
  //   double ajustedVerticalAngle = getVerticalAngle() - verticalTarget;
  //   double verticalAngleOffset = ajustedVerticalAngle / Constants.CoralHandler.verticalRotationDegreesPerRotation;
  //   double neededAngle = verticalMotor.getEncoder().getPosition() + verticalAngleOffset;
  //   verticalMotorController.setReference(neededAngle, ControlType.kMAXMotionPositionControl);
  // }
  
  // public void setHorizontalPosition(Rotation2d targetHorizontalPosition) {
  //   horizontalTarget = targetHorizontalPosition.getDegrees();
  //   double ajustedHorizontalAngle = getVerticalAngle() - horizontalTarget;
  //   double horizontalAngleOffset = ajustedHorizontalAngle / Constants.CoralHandler.horizontalRotationDegreesPerRotation;
  //   double neededAngle = horizontalMotor.getEncoder().getPosition() + horizontalAngleOffset;
  //   horizontalMotorController.setReference(neededAngle, ControlType.kMAXMotionPositionControl);
  // }

  //How to get the 2 axis positions into one method? (helper method)
  public double getAngle(StatusSignal<Angle> absoluteAngleEncoder) {
    return absoluteAngleEncoder.getValueAsDouble() * 360;
  }
  public void setPosition (SparkFlex motor, SparkClosedLoopController motorController, Rotation2d targetPosition, StatusSignal<Angle> absoluteAngleEncoder) {
    target = targetPosition.getDegrees();
    double ajustedAngle = getAngle(absoluteAngleEncoder) - target;
    double angleOffset = ajustedAngle / Constants.CoralHandler.RotationDegreesPerRotation;
    double neededAngle = motor.getEncoder().getPosition() + angleOffset;
    motorController.setReference(neededAngle, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.getBoolean("CoralHandler/Has Coral", false);
    SmartDashboard.getNumber("CoralHandler/Vertical Target Angle", verticalTarget);
    SmartDashboard.getNumber("CoralHandler/Horizontal Target Angle", horizontalTarget);
    SmartDashboard.getNumber("CoralHandler/Absolute Vertical Encoder Angle", getAngle(verticalRotationAbsoluteSignal));
    SmartDashboard.getNumber("CoralHandler/Absolute Horizontal Encoder Angle", getAngle(horizontalRotationAbsoluteSignal));
  }

  @Override
  protected Command systemCheckCommand() {
    throw new UnsupportedOperationException("Unimplemented method 'systemCheckCommand'");
  }
}
