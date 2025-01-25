// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Creates a new CoralHandler. */
public class CoralHandler extends AdvancedSubsystem {
  // TODO Change SparkFlex to SparkMax
  private final SparkFlex outtakeMotor;
  private final SparkMax horizontalMotor;
  private final SparkMax verticalMotor;
  private final SparkClosedLoopController horizontalMotorController;
  private final SparkClosedLoopController verticalMotorController;
  private final CANcoder horizontalAbsoluteEncoder;
  private final CANcoder verticalAbsoluteEncoder;
  private final SparkLimitSwitch coralLimitSwitch;  // Type.kNormallyOpen???
  private final RelativeEncoder outtakeEncoder;
  private final RelativeEncoder horizontalEncoder;
  private final RelativeEncoder verticalEncoder;
  private final StatusSignal<Angle> horizontalRotationAbsoluteSignal;
  private final StatusSignal<Angle> verticalRotationAbsoluteSignal;
  private final SparkFlexSim coralHandlerOuttakeSim;
  private final SparkMaxSim coralHandlerHorizontalSim;
  private final SparkMaxSim coralHandlerVerticalSim;
  private double verticalTarget;
  private double horizontalTarget;
  private double target;

  private final FlywheelSim coralHandlerOuttakePhysicsSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), Constants.CoralHandler.outtakeJKgMetersSquared, Constants.CoralHandler.outtakeMotorGearing),
      DCMotor.getNeoVortex(1), Constants.CoralHandler.outtakeMotorGearing);
  
  private final SingleJointedArmSim coralHandlerHorizontalPhysicsSim = new SingleJointedArmSim(DCMotor.getNeo550(1),
      Constants.CoralHandler.horizontalMotorGearing, Constants.CoralHandler.horizontalJKgMetersSquared,
      Constants.CoralHandler.coralEndEffectorLength, Constants.CoralHandler.horizontalMinAngleInRadians,
      Constants.CoralHandler.horizontalMaxAngleInRadians, false,
      Constants.CoralHandler.horizontalStartingAngleInRadians); // ,Constants.CoralHandler.horizontalMotorStdDev);
  
  
  private final SingleJointedArmSim coralHandlerVerticalPhysicsSim = new SingleJointedArmSim(DCMotor.getNeo550(1),
      Constants.CoralHandler.verticalMotorGearing, Constants.CoralHandler.verticalJKgMetersSquared,
      Constants.CoralHandler.coralEndEffectorLength, Constants.CoralHandler.verticalMinAngleInRadians,
      Constants.CoralHandler.verticalMaxAngleInRadians, true, 
      Constants.CoralHandler.verticalStartingAngleInRadians); // ,Constants.CoralHandler.verticalMotorStdDev);
  
  private final SparkAbsoluteEncoderSim coralHandlerHorizontalAbsoluteEncoderSim;
  private final SparkAbsoluteEncoderSim coralHandlerVerticalAbsoluteEncoderSim;

  public CoralHandler() {
    // Creation of Motors, Encoders, and Limitswitch for CoralHandler Subsystem
    outtakeMotor = new SparkFlex(Constants.CoralHandler.outtakeMotorID, MotorType.kBrushless);
    horizontalMotor = new SparkMax(Constants.CoralHandler.horizontalMotorID, MotorType.kBrushless);
    verticalMotor = new SparkMax(Constants.CoralHandler.verticalMotorID, MotorType.kBrushless);

    outtakeEncoder = outtakeMotor.getEncoder();
    horizontalEncoder = horizontalMotor.getEncoder();
    verticalEncoder = verticalMotor.getEncoder();
    
    horizontalMotorController = horizontalMotor.getClosedLoopController();
    verticalMotorController = verticalMotor.getClosedLoopController();
    
    horizontalAbsoluteEncoder = new CANcoder(Constants.CoralHandler.horizontalEncoderID);
    verticalAbsoluteEncoder = new CANcoder(Constants.CoralHandler.verticalMotorEncoderID);
    
    coralLimitSwitch = outtakeMotor.getForwardLimitSwitch();
    
    coralHandlerHorizontalAbsoluteEncoderSim = new SparkAbsoluteEncoderSim(horizontalMotor);
    coralHandlerVerticalAbsoluteEncoderSim = new SparkAbsoluteEncoderSim(verticalMotor);


    // Using SparkFlexConfig to create needed parameters for the outtakeMotor
    SparkFlexConfig outtakeConfig = new SparkFlexConfig();
    outtakeConfig.inverted(false);
    outtakeConfig.idleMode(IdleMode.kBrake);
    outtakeMotor.configure(outtakeConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    // Creation of configuration for the soft limit for the horizontal motor
    SoftLimitConfig horizontalMotorLimitConfig = new SoftLimitConfig();
    horizontalMotorLimitConfig.forwardSoftLimitEnabled(false);
    horizontalMotorLimitConfig.reverseSoftLimitEnabled(false);

    horizontalMotorLimitConfig
        .forwardSoftLimit((float) (100.0 / Constants.CoralHandler.horizontalRotationDegreesPerRotation));
    horizontalMotorLimitConfig
        .reverseSoftLimit((float) (-100.0 / Constants.CoralHandler.horizontalRotationDegreesPerRotation));

    SparkMaxConfig horizontalMotorConfig = new SparkMaxConfig();
    horizontalMotorConfig.inverted(false);
    horizontalMotorConfig.idleMode(IdleMode.kBrake);

    ClosedLoopConfig horizontalMotorPIDConfig = horizontalMotorConfig.closedLoop;
    horizontalMotorPIDConfig.pid(Constants.CoralHandler.horizontalMotorP, Constants.CoralHandler.horizontalMotorI,
        Constants.CoralHandler.horizontalMotorD);
    horizontalMotorPIDConfig.velocityFF(Constants.CoralHandler.horizontalMotorFeedForward);
    horizontalMotorPIDConfig.iZone(Constants.CoralHandler.horizontalMotorIZone);
    horizontalMotorPIDConfig.maxMotion.maxAcceleration(Constants.CoralHandler.horizontalMotorMaxAccleration);
    horizontalMotorPIDConfig.maxMotion.maxVelocity(Constants.CoralHandler.horizontalMotorMaxVelocity);
    horizontalMotorPIDConfig.maxMotion.allowedClosedLoopError(Constants.CoralHandler.horizontalMotorClosedLoopError);
    horizontalMotor.configure(horizontalMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    CANcoderConfiguration horizontalEncoderConfig = new CANcoderConfiguration();
    horizontalEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    horizontalEncoderConfig.MagnetSensor.MagnetOffset = Preferences.getDouble("horizontalRotationalOffset", 0);
    horizontalAbsoluteEncoder.getConfigurator().apply(horizontalEncoderConfig);
    horizontalRotationAbsoluteSignal = horizontalAbsoluteEncoder.getAbsolutePosition();
    horizontalRotationAbsoluteSignal.refresh();

    SoftLimitConfig verticalLimitConfig = new SoftLimitConfig();
    verticalLimitConfig.forwardSoftLimitEnabled(false);
    verticalLimitConfig.reverseSoftLimitEnabled(false);
    verticalLimitConfig.forwardSoftLimit((float) (90.0 / Constants.CoralHandler.verticalRotationDegreesPerRotation));
    verticalLimitConfig.forwardSoftLimit((float) (-90.0 / Constants.CoralHandler.verticalRotationDegreesPerRotation));

    SparkMaxConfig verticalMotorConfig = new SparkMaxConfig();
    verticalMotorConfig.inverted(false);
    verticalMotorConfig.idleMode(IdleMode.kBrake);

    ClosedLoopConfig verticalMotorPIDConfig = verticalMotorConfig.closedLoop;
    verticalMotorPIDConfig.pid(Constants.CoralHandler.verticalMotorP, Constants.CoralHandler.verticalMotorI,
        Constants.CoralHandler.verticalMotorD);
    verticalMotorPIDConfig.velocityFF(Constants.CoralHandler.verticalMotorFeedForward);
    verticalMotorPIDConfig.iZone(Constants.CoralHandler.verticalMotorIZone);
    verticalMotorPIDConfig.maxMotion.maxAcceleration(Constants.CoralHandler.verticalMotorMaxAccleration);
    verticalMotorPIDConfig.maxMotion.maxVelocity(Constants.CoralHandler.verticalMotorMaxVelocity);
    verticalMotorPIDConfig.maxMotion.allowedClosedLoopError(Constants.CoralHandler.verticalMotorClosedLoopError);
    verticalMotor.configure(verticalMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    CANcoderConfiguration verticalEncoderConfig = new CANcoderConfiguration();
    verticalEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    verticalEncoderConfig.MagnetSensor.MagnetOffset = Preferences.getDouble("verticalRotationalOffset", 0);
    verticalAbsoluteEncoder.getConfigurator().apply(verticalEncoderConfig);
    verticalRotationAbsoluteSignal = verticalAbsoluteEncoder.getAbsolutePosition();
    verticalRotationAbsoluteSignal.refresh();

    // Creation of CoralHandler motor simulation
    coralHandlerOuttakeSim = new SparkFlexSim(outtakeMotor, DCMotor.getNeoVortex(1));
    coralHandlerHorizontalSim = new SparkMaxSim(horizontalMotor, DCMotor.getNeo550(1));
    coralHandlerVerticalSim = new SparkMaxSim(verticalMotor, DCMotor.getNeo550(1));

    registerHardware("Coral Intake/Outtake Motor", outtakeMotor);
    registerHardware("Coral Horizontal Motor", horizontalMotor);
    registerHardware("Coral Horizontal Encoder", horizontalAbsoluteEncoder);
    registerHardware("Coral Vertical Motor", verticalMotor);
    registerHardware("Coral Vertical Encoder", verticalAbsoluteEncoder);
  }
@Override
  public void simulationPeriodic() {
    double outtakeInputVoltage = coralHandlerOuttakeSim.getAppliedOutput() * RobotController.getBatteryVoltage();
    double horizontalInputVoltage = coralHandlerHorizontalSim.getAppliedOutput() * RobotController.getBatteryVoltage();
    double verticalInputVoltage = coralHandlerVerticalSim.getAppliedOutput() * RobotController.getBatteryVoltage();

    coralHandlerOuttakeSim.getForwardLimitSwitchSim().setPressed(false);

    coralHandlerOuttakePhysicsSim.setInput(outtakeInputVoltage);
    coralHandlerHorizontalPhysicsSim.setInput(horizontalInputVoltage);
    coralHandlerVerticalPhysicsSim.setInput(verticalInputVoltage);
    coralHandlerOuttakePhysicsSim.update(0.02);
    coralHandlerHorizontalPhysicsSim.update(0.02);
    coralHandlerVerticalPhysicsSim.update(0.02);

    double outtakeMotorVelocity = coralHandlerOuttakePhysicsSim.getAngularVelocityRPM()
        / Constants.CoralHandler.outtakeMotorGearing;
    double horizontalMotorVelocity = (((coralHandlerHorizontalPhysicsSim.getVelocityRadPerSec() / (2 * Math.PI)) * 60) / Constants.CoralHandler.horizontalMotorGearing);
    double verticalMotorVelocity = (((coralHandlerVerticalPhysicsSim.getVelocityRadPerSec() / (2 * Math.PI)) * 60) / Constants.CoralHandler.verticalMotorGearing);

    coralHandlerOuttakeSim.iterate(outtakeMotorVelocity, RobotController.getBatteryVoltage(), 0.02);
    coralHandlerHorizontalSim.iterate(horizontalMotorVelocity, RobotController.getBatteryVoltage(), 0.02);
    coralHandlerVerticalSim.iterate(verticalMotorVelocity, RobotController.getBatteryVoltage(), 0.02);
    coralHandlerHorizontalAbsoluteEncoderSim.iterate(horizontalMotorVelocity, 0.02);
    coralHandlerVerticalAbsoluteEncoderSim.iterate(verticalMotorVelocity, 0.02);

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(coralHandlerOuttakeSim.getMotorCurrent(), coralHandlerHorizontalSim.getMotorCurrent(), coralHandlerVerticalSim.getMotorCurrent()));
    
    SmartDashboard.putNumber("Horizontal Absolute Encoder Sim",
        coralHandlerHorizontalSim.getAbsoluteEncoderSim().getPosition());
    SmartDashboard.putNumber("Vertical Absolute Encoder Sim",
        coralHandlerVerticalSim.getAbsoluteEncoderSim().getPosition());
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
  private void setVerticalPosition(SparkMax motor, SparkClosedLoopController motorController, Rotation2d targetPosition,
      StatusSignal<Angle> absoluteAngleEncoder) {
    target = MathUtil.clamp(targetPosition.getDegrees(), -90, 90);
    double ajustedAngle = getAngle(absoluteAngleEncoder) - target;
    double angleOffset = ajustedAngle / Constants.CoralHandler.verticalRotationDegreesPerRotation;
    double neededAngle = motor.getEncoder().getPosition() + angleOffset;
    motorController.setReference(neededAngle, ControlType.kMAXMotionPositionControl);
  }

  private void setHorizontalPosition(SparkMax motor, SparkClosedLoopController motorController,
      Rotation2d targetPosition,
      StatusSignal<Angle> absoluteAngleEncoder) {
    target = MathUtil.clamp(targetPosition.getDegrees(), -100, 100);
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
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              outtakeMotor.set(0.25);
              horizontalMotor.set(0.25);
              verticalMotor.set(.75);
            }, this),
        Commands.waitSeconds(10),
        Commands.runOnce(
            () -> {
              if ((outtakeEncoder.getVelocity()) < Constants.CoralHandler.outtakeMotorMinVelocity) {
                addFault("[System Check] Outtake Coral Motor too slow (forward direction)", false, true);
              }
              if ((horizontalEncoder.getVelocity()) < Constants.CoralHandler.horizontalMotorMinVelocity) {
                addFault("[System Check] Horizontal Coral Motor too slow (forward direction)", false, true);
              }
              if ((verticalEncoder.getVelocity()) < Constants.CoralHandler.verticalMotorMinVelocity) {
                addFault("[System Check] Vertical Coral Motor too slow (forward direction)", false, true);
              }
            }, this),
        Commands.runOnce(
            () -> {
              outtakeMotor.set(-0.25);
              horizontalMotor.set(-0.25);
              verticalMotor.set(-0.25);
            }, this),
        Commands.waitSeconds(10),
        Commands.runOnce(
            () -> {
              if ((outtakeEncoder.getVelocity()) < -Constants.CoralHandler.outtakeMotorMinVelocity) {
                addFault("[System Check] Outtake Coral Motor too slow (forward direction)", false, true);
              }
              if ((horizontalEncoder.getVelocity()) < -Constants.CoralHandler.horizontalMotorMinVelocity) {
                addFault("[System Check] Horizontal Coral Motor too slow (backwards direction)", false, true);
              }
              if ((verticalEncoder.getVelocity()) < -Constants.CoralHandler.verticalMotorMinVelocity) {
                addFault("[System Check] Vertical Coral Motor too slow (backwards direction)", false, true);
              }
              stopMotors();
            }, this));
  }
}