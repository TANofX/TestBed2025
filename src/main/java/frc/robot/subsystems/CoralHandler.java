// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

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
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
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

/** Creates a new CoralHandler. */
public class CoralHandler extends AdvancedSubsystem {
  private final SparkFlex outtakeMotor;
  private final SparkMax horizontalMotor;
  private final SparkMax verticalMotor;
  private final SparkClosedLoopController horizontalMotorController;
  private final SparkClosedLoopController verticalMotorController;
  private final CANcoder horizontalAbsoluteEncoder;
  private final CANcoder verticalAbsoluteEncoder;
  private final SparkLimitSwitch coralLimitSwitch;
  private final RelativeEncoder outtakeEncoder;
  private final RelativeEncoder horizontalEncoder;
  private final RelativeEncoder verticalEncoder;
  private final StatusSignal<Angle> horizontalRotationAbsoluteSignal;
  private final StatusSignal<Angle> verticalRotationAbsoluteSignal;
  private final SparkFlexSim coralHandlerOuttakeSim;
  private final SparkMaxSim coralHandlerHorizontalSim;
  private final SparkMaxSim coralHandlerVerticalSim;
  private double target;

  // Creation of Flywheel Simulation for the simulation of the outtakeMotor
  private final FlywheelSim coralHandlerOuttakePhysicsSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), 
                                          Constants.CoralHandler.outtakeJKgMetersSquared, 
                                          Constants.CoralHandler.outtakeMotorGearing),
      DCMotor.getNeoVortex(1), 
      Constants.CoralHandler.outtakeMotorGearing);

  // Creation of SingleJoinedArm Sumulatiion of the simulation of the horizontalMotor
  private final SingleJointedArmSim coralHandlerHorizontalPhysicsSim = new SingleJointedArmSim(DCMotor.getNeo550(1),
      Constants.CoralHandler.horizontalMotorGearing, Constants.CoralHandler.horizontalJKgMetersSquared,
      Constants.CoralHandler.coralEndEffectorLength, Constants.CoralHandler.horizontalMinAngleInRadians,
      Constants.CoralHandler.horizontalMaxAngleInRadians, true,
      Constants.CoralHandler.horizontalStartingAngleInRadians); // ,Constants.CoralHandler.horizontalMotorStdDev);

  // Creation of SingleJoinedArm Sumulatiion of the simulation of the verticalMotor
  private final SingleJointedArmSim coralHandlerVerticalPhysicsSim = new SingleJointedArmSim(DCMotor.getNeo550(1),
      Constants.CoralHandler.verticalMotorGearing, Constants.CoralHandler.verticalJKgMetersSquared,
      Constants.CoralHandler.coralEndEffectorLength, Constants.CoralHandler.verticalMinAngleInRadians,
      Constants.CoralHandler.verticalMaxAngleInRadians, false,
      Constants.CoralHandler.verticalStartingAngleInRadians); // ,Constants.CoralHandler.verticalMotorStdDev);

  private final SparkAbsoluteEncoderSim coralHandlerHorizontalAbsoluteEncoderSim;
  private final SparkAbsoluteEncoderSim coralHandlerVerticalAbsoluteEncoderSim;

  public CoralHandler(int outtakeMotorID, int horizontalMotorID, int verticalMotorID, int horizontalAbsoluteEncoderID,
      int verticalAbsoluteEncoderID) {
    // Creation of Motors, Encoders, and Limitswitch for CoralHandler Subsystem
    outtakeMotor = new SparkFlex(outtakeMotorID, MotorType.kBrushless);
    horizontalMotor = new SparkMax(horizontalMotorID, MotorType.kBrushless);
    verticalMotor = new SparkMax(verticalMotorID, MotorType.kBrushless);

    horizontalMotorController = horizontalMotor.getClosedLoopController();
    verticalMotorController = verticalMotor.getClosedLoopController();

    outtakeEncoder = outtakeMotor.getEncoder();
    horizontalEncoder = horizontalMotor.getEncoder();
    verticalEncoder = verticalMotor.getEncoder();

    horizontalAbsoluteEncoder = new CANcoder(horizontalAbsoluteEncoderID);
    verticalAbsoluteEncoder = new CANcoder(verticalAbsoluteEncoderID);

    coralLimitSwitch = outtakeMotor.getForwardLimitSwitch(); // TODO forward or reverse limit switch?

    coralHandlerHorizontalAbsoluteEncoderSim = new SparkAbsoluteEncoderSim(horizontalMotor);
    coralHandlerVerticalAbsoluteEncoderSim = new SparkAbsoluteEncoderSim(verticalMotor);

    // Using SparkFlexConfig to create needed parameters for the outtakeMotor
    SparkFlexConfig outtakeConfig = new SparkFlexConfig();
    outtakeConfig.inverted(false);
    outtakeConfig.idleMode(IdleMode.kBrake);
    outtakeMotor.configure(outtakeConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    // Creation of needed settings of limitswitch for the horizontalMotor
    LimitSwitchConfig horizontalLimitConfig = new LimitSwitchConfig(); // TODO do we need the normal limit switch
                                                                       // config?
    horizontalLimitConfig.forwardLimitSwitchType(Type.kNormallyOpen);

    // Creation of configuration for the soft limit for the horizontal motor
    SoftLimitConfig horizontalMotorLimitConfig = new SoftLimitConfig();
    horizontalMotorLimitConfig.forwardSoftLimitEnabled(false); // TODO possibly enable the soft limit on limit switch?
    horizontalMotorLimitConfig.reverseSoftLimitEnabled(false); // TODO possibly enable the soft limit on limit switch?

    horizontalMotorLimitConfig
        .forwardSoftLimit((float) (100.0 / Constants.CoralHandler.horizontalRotationDegreesPerRotation));
    horizontalMotorLimitConfig
        .reverseSoftLimit((float) (-100.0 / Constants.CoralHandler.horizontalRotationDegreesPerRotation));

    // Creation of motorConfig for horizontalMotor
    SparkMaxConfig horizontalMotorConfig = new SparkMaxConfig();
    horizontalMotorConfig.inverted(false);
    horizontalMotorConfig.idleMode(IdleMode.kBrake);

    // Creation of PID + others settings for horizontalMotor
    ClosedLoopConfig horizontalMotorPIDConfig = horizontalMotorConfig.closedLoop;
    horizontalMotorPIDConfig.pidf(Constants.CoralHandler.horizontalMotorP, Constants.CoralHandler.horizontalMotorI,
        Constants.CoralHandler.horizontalMotorD, Constants.CoralHandler.horizontalMotorFeedForward);
    horizontalMotorPIDConfig.iZone(Constants.CoralHandler.horizontalMotorIZone);
    horizontalMotorPIDConfig.maxMotion.maxAcceleration(Constants.CoralHandler.horizontalMotorMaxAccleration);
    horizontalMotorPIDConfig.maxMotion.maxVelocity(Constants.CoralHandler.horizontalMotorMaxVelocity);
    horizontalMotorPIDConfig.maxMotion.allowedClosedLoopError(Constants.CoralHandler.horizontalMotorClosedLoopError);
    horizontalMotorConfig.apply(horizontalMotorPIDConfig);
    horizontalMotor.configure(horizontalMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    // Creation of encoderConfig for the horizontal motor absolute encoder
    CANcoderConfiguration horizontalEncoderConfig = new CANcoderConfiguration();
    horizontalEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    horizontalEncoderConfig.MagnetSensor.MagnetOffset = Preferences.getDouble("horizontalRotationalOffset", 0);
    horizontalAbsoluteEncoder.getConfigurator().apply(horizontalEncoderConfig);
    horizontalAbsoluteEncoder.getConfigurator().setPosition(0); // TODO set position of Encoder
    horizontalRotationAbsoluteSignal = horizontalAbsoluteEncoder.getAbsolutePosition();
    horizontalRotationAbsoluteSignal.refresh();

    // Creation of needed settings of limitswitch for the horizontalMotor
    LimitSwitchConfig verticalLimitConfig = new LimitSwitchConfig(); // TODO do we need the normal limit switch config?
    verticalLimitConfig.forwardLimitSwitchType(Type.kNormallyOpen);

    // Creation of configuration for the soft limit for the vertical motor
    SoftLimitConfig verticalSoftLimitConfig = new SoftLimitConfig();
    verticalSoftLimitConfig.forwardSoftLimitEnabled(false); // TODO possibly enable the soft limit on limit switches?
    verticalSoftLimitConfig.reverseSoftLimitEnabled(false); // TODO possibly enable the soft limit on limit switches?

    verticalSoftLimitConfig
        .forwardSoftLimit((float) (90.0 / Constants.CoralHandler.verticalRotationDegreesPerRotation));
    verticalSoftLimitConfig
        .forwardSoftLimit((float) (-90.0 / Constants.CoralHandler.verticalRotationDegreesPerRotation));

    // Creation of motorConfig for verticalMotor
    SparkMaxConfig verticalMotorConfig = new SparkMaxConfig();
    verticalMotorConfig.inverted(false);
    verticalMotorConfig.idleMode(IdleMode.kBrake);

    // Creation of PID + other settings for verticalMotor
    ClosedLoopConfig verticalMotorPIDConfig = verticalMotorConfig.closedLoop;
    verticalMotorPIDConfig.pidf(Constants.CoralHandler.verticalMotorP, Constants.CoralHandler.verticalMotorI,
        Constants.CoralHandler.verticalMotorD, Constants.CoralHandler.verticalMotorFeedForward);
    verticalMotorPIDConfig.iZone(Constants.CoralHandler.verticalMotorIZone);
    verticalMotorPIDConfig.maxMotion.maxAcceleration(Constants.CoralHandler.verticalMotorMaxAccleration);
    verticalMotorPIDConfig.maxMotion.maxVelocity(Constants.CoralHandler.verticalMotorMaxVelocity);
    verticalMotorPIDConfig.maxMotion.allowedClosedLoopError(Constants.CoralHandler.verticalMotorClosedLoopError);
    verticalMotorConfig.apply(verticalMotorPIDConfig);
    verticalMotor.configure(verticalMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);


    // Creation of encoderConfig for the vertical motor absolute encoder
    CANcoderConfiguration verticalEncoderConfig = new CANcoderConfiguration();
    verticalEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    verticalEncoderConfig.MagnetSensor.MagnetOffset = Preferences.getDouble("verticalRotationalOffset", 0);
    verticalAbsoluteEncoder.getConfigurator().apply(verticalEncoderConfig);
    verticalAbsoluteEncoder.getConfigurator().setPosition(0); // TODO set position of Encoder
    verticalRotationAbsoluteSignal = verticalAbsoluteEncoder.getAbsolutePosition();
    verticalRotationAbsoluteSignal.refresh();

    // Creation of CoralHandler motor simulation
    coralHandlerOuttakeSim = new SparkFlexSim(outtakeMotor, DCMotor.getNeoVortex(1));
    coralHandlerHorizontalSim = new SparkMaxSim(horizontalMotor, DCMotor.getNeo550(1));
    coralHandlerVerticalSim = new SparkMaxSim(verticalMotor, DCMotor.getNeo550(1));

    // Register Hardware
    registerHardware("Coral Intake/Outtake Motor", outtakeMotor);
    registerHardware("Coral Horizontal Motor", horizontalMotor);
    registerHardware("Coral Horizontal Encoder", horizontalAbsoluteEncoder);
    registerHardware("Coral Vertical Motor", verticalMotor);
    registerHardware("Coral Vertical Encoder", verticalAbsoluteEncoder);
  }

  @Override
  public void simulationPeriodic() {
    // Simulates the input voltage of the motors (battery)
    double outtakeInputVoltage = coralHandlerOuttakeSim.getAppliedOutput() * RobotController.getBatteryVoltage();
    double horizontalInputVoltage = coralHandlerHorizontalSim.getAppliedOutput() * RobotController.getBatteryVoltage();
    double verticalInputVoltage = coralHandlerVerticalSim.getAppliedOutput() * RobotController.getBatteryVoltage();

    // Simulation limit switch is set to false
    coralHandlerOuttakeSim.getForwardLimitSwitchSim().setPressed(false);

    // Sets the simulation input velocities based on the voltages above
    coralHandlerOuttakePhysicsSim.setInput(outtakeInputVoltage);
    coralHandlerHorizontalPhysicsSim.setInput(horizontalInputVoltage);
    coralHandlerVerticalPhysicsSim.setInput(verticalInputVoltage);

    // Simulates time by updating the time
    coralHandlerOuttakePhysicsSim.update(0.02);
    coralHandlerHorizontalPhysicsSim.update(0.02);
    coralHandlerVerticalPhysicsSim.update(0.02);

    // Calculating the simulation velocity based on known values
    double outtakeMotorVelocity = coralHandlerOuttakePhysicsSim.getAngularVelocityRPM()
        / Constants.CoralHandler.outtakeMotorGearing;
    double horizontalMotorVelocity = (coralHandlerHorizontalPhysicsSim.getVelocityRadPerSec() / (2 * Math.PI) * 60)
        / Constants.CoralHandler.horizontalMotorGearing;
    double verticalMotorVelocity = (coralHandlerVerticalPhysicsSim.getVelocityRadPerSec() / (2 * Math.PI) * 60)
        / Constants.CoralHandler.verticalMotorGearing;

    // Creation of the motor simulations
    coralHandlerOuttakeSim.iterate(outtakeMotorVelocity, RobotController.getBatteryVoltage(), 0.02);
    coralHandlerHorizontalSim.iterate(horizontalMotorVelocity, RobotController.getBatteryVoltage(), 0.02);
    coralHandlerVerticalSim.iterate(verticalMotorVelocity, RobotController.getBatteryVoltage(), 0.02);

    // Creation of the absolute encoder simulations
    coralHandlerHorizontalAbsoluteEncoderSim.iterate((horizontalMotorVelocity * Constants.CoralHandler.horizontalMotorGearing), 0.02); // TODO what velocity am I supposed to put here, ik its supposed to be from the physics sim itself but weh?
    coralHandlerVerticalAbsoluteEncoderSim.iterate((verticalMotorVelocity * Constants.CoralHandler.verticalMotorGearing), 0.02); // TODO what velocity am I supposed to put here, ik its supposed to be from the physics sim itself but weh?

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(coralHandlerOuttakeSim.getMotorCurrent(),
        coralHandlerHorizontalSim.getMotorCurrent(), coralHandlerVerticalSim.getMotorCurrent()));

    SmartDashboard.putNumber("Horizontal Absolute Encoder Sim Angle",
        coralHandlerHorizontalSim.getAbsoluteEncoderSim().getPosition());
    SmartDashboard.putNumber("Vertical Absolute Encoder Sim Angle",
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
   * Obtains the angle from the Absolute Encoder wanted on the coral end effector.
   * 
   * @param absoluteEncoder Specify which coral end effector encoder should
   *                        be read.
   * @return Angle as rotation
   */
  public StatusSignal<Angle> getAbsoluteEncoderAngle(CANcoder absoluteEncoder) {
    return absoluteEncoder.getAbsolutePosition(); // TODO Is it absolute position? also change @return
  }

  /**
   * Sets the angle of the needed motor (horizontal/vertical) for the Coral
   * Handler.
   * 
   * @param motor                      Specify which motor needs to be read.
   * @param motorController            The motorController that is correlated to the motor.
   * @param targetAngle                The needed angle to be obtained.
   * @param absoluteEncoder            The correlated encoder to the specified motor.
   * @param clampMin                   The lowest angle that can be set to the motor.
   * @param clampMax                   The highest angle that can be set to the motor.
   * @param rotationDegreesPerRotation The constants value of the related motor.
   */
    
    private void setAngle(SparkMax motor, SparkClosedLoopController motorController, Rotation2d targetAngle, CANcoder absoluteEncoder, double clampMin, double clampMax, double rotationDegreesPerRotation) {
    //the target rotation of the arm in degrees
    target = MathUtil.clamp(targetAngle.getDegrees(), clampMin, clampMax);
    //how much is wanted to move in degrees
    double adjustedAngle = getAbsoluteEncoderAngle(absoluteEncoder).getValue().in(Degrees) - target; // TODO This is supposed to return double bc it was degrees...?
    //how far to rotate the motor in degrees from where we are currently
    double angleOffset = adjustedAngle / rotationDegreesPerRotation;
    //the actual target angle --> the target absolute rotation of the motor
    double neededAngle = Units.rotationsToDegrees(motor.getEncoder().getPosition()) + angleOffset;
    motorController.setReference(neededAngle, ControlType.kMAXMotionPositionControl); // TODO Is it supposed be position control?
  }

  /**
   * Sets the horizontal positioning of the coral end effector to a specified
   * angle.
   * 
   * @param targetAngle The desired horizontal angle (degrees) for the coral end
   *                    effector.
   */
  public void setHorizontalAngle(Rotation2d targetAngle) {
    setAngle(horizontalMotor, horizontalMotorController, targetAngle, horizontalAbsoluteEncoder, -100.0, 100.0,
        Constants.CoralHandler.horizontalRotationDegreesPerRotation);
  }

  /**
   * Sets the verical positioning of the coral end effector to a specified angle.
   * 
   * @param targetAngle The desired vertical angle (degrees) for the coral end
   *                    effector.
   */
  public void setVerticalAngle(Rotation2d targetAngle) {
    setAngle(verticalMotor, verticalMotorController, targetAngle, verticalAbsoluteEncoder, -90, 90,
        Constants.CoralHandler.verticalRotationDegreesPerRotation);
  }

  @Override
  public void periodic() {
    // Getting the fresh signal/angle from the absolute encoder
    horizontalRotationAbsoluteSignal.refresh();
    verticalRotationAbsoluteSignal.refresh();

    // Values avalible shown on SmartDashboard
    SmartDashboard.getBoolean("CoralHandler/Has Coral", false);
  }

  public Command runCoralIntakeCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> runOuttakeMotor(Constants.CoralHandler.coralIntakeSpeed), this),
        Commands.waitUntil(
            () -> hasCoral()),
        Commands.runOnce(
            () -> stopOuttakeMotor(), this));
  }

  public Command runCoralOuttakeCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> runOuttakeMotor(Constants.CoralHandler.coralOuttakeSpeed), this),
        Commands.waitUntil(
            () -> !hasCoral()),
        Commands.waitSeconds(.5),
        Commands.runOnce(
            () -> stopOuttakeMotor(), this));
  }

  public Command setVerticalAngleCommand(Rotation2d vTargetAngle) {
      SmartDashboard.putNumber("vTargetAngle in Degrees", vTargetAngle.getDegrees());
      SmartDashboard.putNumber("vTargetAngle in Radians", vTargetAngle.getRadians());
      SmartDashboard.putNumber("vTargetAngle in Rptations", vTargetAngle.getRotations());
      SmartDashboard.putNumber("Difference in Angle", (Math.abs(vTargetAngle.getDegrees() - getAbsoluteEncoderAngle(verticalAbsoluteEncoder).getValue().in(Degrees))));
      SmartDashboard.putNumber("Encoder Angle ", getAbsoluteEncoderAngle(verticalAbsoluteEncoder).getValue().in(Degrees));
      return Commands.sequence(
        Commands.runOnce(() -> {
          setVerticalAngle(vTargetAngle);
        }, this),
        Commands.waitUntil(()->{
          return Math.abs(vTargetAngle.getDegrees() - getAbsoluteEncoderAngle(verticalAbsoluteEncoder).getValue().in(Degrees)) < 2;
        }),
        Commands.runOnce(() -> {
          verticalMotor.set(0);
        }, this)
      );
  }

@Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              runOuttakeMotor(1);
              // setHorizontalAngle(Rotation2d.fromDegrees(10)); // TODO Is this a way to set the target angle, because it is a rotation2d?
              setVerticalAngle(Rotation2d.fromDegrees(10)); // TODO Is this a way to set the target angle, because it is a rotation2d?
            }, this),
        Commands.waitSeconds(1.0),
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
              runOuttakeMotor(-1);
              // setHorizontalAngle(Rotation2d.fromDegrees(0)); // TODO Is this a way to set the target angle, because it is a rotation2d?
              setVerticalAngle(Rotation2d.fromDegrees(0)); // TODO Is this a way to set the target angle, because it is a rotation2d?
            }, this),
        Commands.waitSeconds(1.0),
        Commands.runOnce(
            () -> {
              if ((outtakeEncoder.getVelocity()) < -Constants.CoralHandler.outtakeMotorMinVelocity) {
                addFault("[System Check] Outtake Coral Motor too slow (backwards direction)", false, true);
              }
              if ((horizontalEncoder.getVelocity()) < -Constants.CoralHandler.horizontalMotorMinVelocity) {
                addFault("[System Check] Horizontal Coral Motor too slow (backwards direction)", false, true);
              }
              if ((verticalEncoder.getVelocity()) < -Constants.CoralHandler.verticalMotorMinVelocity) {
                addFault("[System Check] Vertical Coral Motor too slow (backwards direction)", false, true);
              }
              horizontalMotor.set(0);
            }, this));
  }
}