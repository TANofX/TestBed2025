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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

/** Creates a new CoralHandler. */
public class CoralHandler extends AdvancedSubsystem {
  //TODO Change SparkFlex to SparkMax
  // Creation of Motors, Encoders, and Limitswitch for CoralHandler Subsystem
  private final SparkFlex outtakeMotor = new SparkFlex(Constants.CoralHandler.outtakeMotorID, MotorType.kBrushless);
  private final SparkMax horizontalMotor = new SparkMax(Constants.CoralHandler.horizontalMotorID, MotorType.kBrushless);
  private final SparkClosedLoopController horizontalMotorController = horizontalMotor.getClosedLoopController();
  private final CANcoder horizontalEncoder = new CANcoder(Constants.CoralHandler.horizontalEncoderID);
  private final SparkFlex verticalMotor = new SparkFlex(Constants.CoralHandler.verticalMotorID, MotorType.kBrushless);
  private final SparkClosedLoopController verticalMotorController = verticalMotor.getClosedLoopController();
  private final CANcoder verticalEncoder = new CANcoder(Constants.CoralHandler.verticalMotorEncoderID);
  private final SparkLimitSwitch coralLimitSwitch = outtakeMotor.getForwardLimitSwitch();
  // Type.kNormallyOpen???
  private final StatusSignal<Angle> horizontalRotationAbsoluteSignal;
  private final StatusSignal<Angle> verticalRotationAbsoluteSignal;
  private double verticalTarget;
  private double horizontalTarget;
  private double target;
  
  private final SingleJointedArmSim coralHandlerVerticalSim = new SingleJointedArmSim(DCMotor.getNeoVortex(1), Constants.CoralHandler.verticalMotorGearing, Constants.CoralHandler.verticalJKgMetersSquared, Constants.CoralHandler.coralEndEffectorLength, Constants.CoralHandler.verticalMinAngleInRadians, Constants.CoralHandler.verticalMaxAngleInRadians, true, Constants.CoralHandler.verticalStartingAngleInRadians, Constants.CoralHandler.verticalMotorStdDev);
  private final SingleJointedArmSim coralHandlerHorizontalSim = new SingleJointedArmSim(DCMotor.getNeo550(1), Constants.CoralHandler.horizontalMotorGearing, Constants.CoralHandler.horizontalJKgMetersSquared, Constants.CoralHandler.coralEndEffectorLength, Constants.CoralHandler.horizontalMinAngleInRadians, Constants.CoralHandler.horizontalMaxAngleInRadians, false, Constants.CoralHandler.horizontalStartingAngleInRadians, Constants.CoralHandler.horizontalMotorStdDev);
  private final FlywheelSim coralHandlerOuttakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), Constants.CoralHandler.outtakeJKgMetersSquared, horizontalTarget), DCMotor.getNeoVortex(1), Constants.CoralHandler.outtakeMotorGearing);
  
  public CoralHandler() {
    // Using SparkFlexConfig, ClosedLoopConfig (also called PIDConfig), and CANcoderConfig to input the needed parameters for Coral Handler Motors and Encoders
    SparkFlexConfig outtakeConfig = new SparkFlexConfig();
    outtakeConfig.inverted(false);
    outtakeConfig.idleMode(IdleMode.kBrake);
    outtakeMotor.configure(outtakeConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    SparkMaxConfig horizontalMotorConfig = new SparkMaxConfig();
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
    
    registerHardware("Coral Intake/Outtake Motor", outtakeMotor);
    registerHardware("Coral Horizontal Motor", horizontalMotor);
    registerHardware("Coral Horizontal Encoder", horizontalEncoder);
    registerHardware("Coral Vertical Motor", verticalMotor);
    registerHardware("Coral Vertical Encoder", verticalEncoder);
  }
  /**
   * Stops motor for the coral end effector intake/outtake motor. Sets motor speed
   * to zero.
   */
  public void stopOuttakeMotor() {
    outtakeMotor.stopMotor();
  }

  /**
   * Stops motor for the coral end effector horizontal motor. Sets motor speed to
   * zero.
   */
  public void stopHorizontalMotor() {
    horizontalMotor.stopMotor();
  }

  /**
   * Stops motor for the coral end effector vertical motor. Sets motor speed to
   * zero.
   */
  public void stopVerticalMotor() {
    verticalMotor.stopMotor();
  }

  /**
   * Stops all motors for the coral end effector. Sets all motor speeds to zero.
   */
  public void stopMotors() {
    outtakeMotor.stopMotor();
    horizontalMotor.stopMotor();
    verticalMotor.stopMotor();
  }

  /**
   * A boolean that will return true or false based on if coral end effector's
   * limit switch is hit.
   * 
   * @return true or false.
   */
  public boolean hasCoral() {
    return coralLimitSwitch.isPressed();
  }

  /**
   * Sets coral intake/outtake motor to a specified speed to shoot the coral out.
   * 
   * @param outtakeMotorSpeed Speed to set to be able outtake the coral using
   *                          coral end effector intake/outtake motor.
   */
  public void runOuttakeMotor(double outtakeMotorSpeed) {
    outtakeMotor.set(outtakeMotorSpeed);
  }

  /**
   * Sets coral intake/outtake motor to a specified speed to intake the coral.
   * 
   * @param intakeMotorSpeed Speed to set to be able outtake the coral using coral
   *                         end effector intake/outtake motor.
   */
  public void runIntakeMotor(double intakeMotorSpeed) {
    outtakeMotor.set(intakeMotorSpeed);
  }

  /**
   * Obtains the angle from the Absolute Encoder wanted on the coral end effector.
   * 
   * @param absoluteAngleEncoder Specify which coral end effector encoder should
   *                             be read.
   * @return Angle as a degree.
   */
  public double getAngle(StatusSignal<Angle> absoluteAngleEncoder) {
    return absoluteAngleEncoder.getValueAsDouble() * 360;
  }

  // Method that sets Position/Angle of horizontal/vertical motors
  private void setVerticalPosition(SparkFlex motor, SparkClosedLoopController motorController, Rotation2d targetPosition,
      StatusSignal<Angle> absoluteAngleEncoder) {
    target = targetPosition.getDegrees();
    double ajustedAngle = getAngle(absoluteAngleEncoder) - target;
    double angleOffset = ajustedAngle / Constants.CoralHandler.verticalRotationDegreesPerRotation;
    double neededAngle = motor.getEncoder().getPosition() + angleOffset;
    motorController.setReference(neededAngle, ControlType.kMAXMotionPositionControl);
  }
  private void setHorizontalPosition(SparkMax motor, SparkClosedLoopController motorController, Rotation2d targetPosition,
      StatusSignal<Angle> absoluteAngleEncoder) {
    target = targetPosition.getDegrees();
    double ajustedAngle = getAngle(absoluteAngleEncoder) - target;
    double angleOffset = ajustedAngle / Constants.CoralHandler.horizontalRotationDegreesPerRotation;
    double neededAngle = motor.getEncoder().getPosition() + angleOffset;
    motorController.setReference(neededAngle, ControlType.kMAXMotionPositionControl);
  }
  /**
   * Sets the horizontal positioning of the coral end effector to a specified
   * angle.
   * 
   * @param targetAngle The desired horizontal angle for the coral end effector.
   */
  public void setHorizontalAngle(Rotation2d targetAngle) {
    setHorizontalPosition(horizontalMotor, horizontalMotorController, targetAngle, horizontalRotationAbsoluteSignal);
  }

  /**
   * Sets the verical positioning of the coral end effector to a specified angle.
   * 
   * @param targetAngle The desired vertical angle for the coral end effector.
   */
  public void setVerticalAngle(Rotation2d targetAngle) {
    setVerticalPosition(verticalMotor, verticalMotorController, targetAngle, verticalRotationAbsoluteSignal);
  }

  @Override
  public void periodic() {
    // Values avalible shown on SmartDashboard
    horizontalRotationAbsoluteSignal.refresh();
    verticalRotationAbsoluteSignal.refresh();
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
