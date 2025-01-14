package frc.lib.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;

/** Implementation for an SDS Mk4 swerve module using Falcon 500s and phoenix pro */
public class Mk4SwerveModulePro extends AdvancedSubsystem {
  public enum ModuleCode {
    FL,
    FR,
    BL,
    BR
  }

  // Volts to meters/sec
  private static final double DRIVE_KV = 3.4;
  // Volts to meters/sec^2
  private static final double DRIVE_KA = 0.27;

  // Volts to deg/sec
  private static final double ROTATION_KV = 12.0 / 900;
  // Volts to deg/sec^2
  private static final double ROTATION_KA = 0.00006;

  private static final double DRIVE_GEARING = 1.0 / 6.75;
  private static final double DRIVE_METERS_PER_ROTATION =
      DRIVE_GEARING * Units.inchesToMeters(12.375); //Math.PI * Units.inchesToMeters(4);
  private static final double ROTATION_DEGREES_PER_ROTATION = (1.0 / 12.8) * 360.0;

  // M/s - Tune (Apply full output and measure max vel. Adjust KV/KA for sim if needed)
  public static final double DRIVE_MAX_VEL = 4.65;

  private static final double DRIVE_KP = 0.3;
  private static final double DRIVE_KD = 0.0;

  private static final double ROTATION_KP = 4.0;
  private static final double ROTATION_KD = 0.0;

  public final ModuleCode moduleCode;
  private final LinearSystemSim<N1, N1, N1> driveSim;
  private final LinearSystemSim<N1, N1, N1> rotationSim;

  private final TalonFX driveMotor;
  private final TalonFXSimState driveSimState;
  private final StatusSignal<Angle> drivePositionSignal;
  private final StatusSignal<AngularVelocity> driveVelocitySignal;
  private final StatusSignal<Current> driveCurrentSignal;
  private final StatusSignal<Voltage> driveVoltageSignal;
  private final StatusSignal<Temperature> driveTempSignal;

  private final TalonFX rotationMotor;
  private final TalonFXSimState rotationSimState;
  private final StatusSignal<Angle> rotationPositionSignal;
  private final StatusSignal<AngularVelocity> rotationVelocitySignal;
  private final StatusSignal<Current> rotationCurrentSignal;
  private final StatusSignal<Voltage> rotationVoltageSignal;
  private final StatusSignal<Temperature> rotationTempSignal;

  private final CANcoder rotationEncoder;
  private final CANcoderConfiguration rotationEncoderConfig;
  private final CANcoderSimState rotationEncoderSimState;
  private final StatusSignal<Angle> rotationAbsoluteSignal;
  private final StatusSignal<AngularVelocity> rotationAbsoluteVelSignal;

  private SwerveModuleState targetState = new SwerveModuleState();

  /**
   * Create a Mk4 swerve module
   *
   * @param moduleCode The code representing this module
   * @param driveMotorCanID The CAN ID of the drive motor
   * @param rotationMotorCanID The CAN ID of the rotation motor
   * @param encoderCanID The CAN ID of the rotation CANCoder
   * @param canBus The name of the can bus the devices are connected to.
   */
  public Mk4SwerveModulePro(
      ModuleCode moduleCode,
      int driveMotorCanID,
      int rotationMotorCanID,
      int encoderCanID,
      String canBus) {
    super(moduleCode.name() + "SwerveModule");

    this.moduleCode = moduleCode;

    rotationEncoder = new CANcoder(encoderCanID, canBus);
    rotationEncoderConfig = new CANcoderConfiguration();
    rotationEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    rotationEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    rotationEncoderConfig.MagnetSensor.MagnetOffset =
        Preferences.getDouble(getName() + "RotationOffset", 0.0) / 360.0;
    rotationEncoder.getConfigurator().apply(rotationEncoderConfig);
    rotationEncoderSimState = rotationEncoder.getSimState();

    driveMotor = new TalonFX(driveMotorCanID, canBus);
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    driveConfig.Slot0.kP = DRIVE_KP;
    driveConfig.Slot0.kD = DRIVE_KD;
    driveConfig.Slot0.kV = 12.0 / (DRIVE_MAX_VEL / DRIVE_METERS_PER_ROTATION);
    driveMotor.getConfigurator().apply(driveConfig);
    driveSimState = driveMotor.getSimState();

    rotationMotor = new TalonFX(rotationMotorCanID, canBus);
    TalonFXConfiguration rotationConfig = new TalonFXConfiguration();
    rotationConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rotationConfig.Slot0.kP = ROTATION_KP;
    rotationConfig.Slot0.kD = ROTATION_KD;
    rotationMotor.getConfigurator().apply(rotationConfig);
    rotationSimState = rotationMotor.getSimState();

    drivePositionSignal = driveMotor.getPosition();
    driveVelocitySignal = driveMotor.getVelocity();
    driveCurrentSignal = driveMotor.getSupplyCurrent();
    driveVoltageSignal = driveMotor.getSupplyVoltage();
    driveTempSignal = driveMotor.getDeviceTemp();

    rotationPositionSignal = rotationMotor.getPosition();
    rotationVelocitySignal = rotationMotor.getVelocity();
    rotationCurrentSignal = rotationMotor.getSupplyCurrent();
    rotationVoltageSignal = rotationMotor.getSupplyVoltage();
    rotationTempSignal = rotationMotor.getDeviceTemp();

    rotationAbsoluteSignal = rotationEncoder.getAbsolutePosition();
    rotationAbsoluteVelSignal = rotationEncoder.getVelocity();

    driveSim = new LinearSystemSim<>(LinearSystemId.identifyVelocitySystem(DRIVE_KV, DRIVE_KA));
    rotationSim =
        new LinearSystemSim(LinearSystemId.identifyPositionSystem(ROTATION_KV, ROTATION_KA));

    registerHardware("Drive Motor", driveMotor);
    registerHardware("Rotation Motor", rotationMotor);
    registerHardware("Rotation Encoder", rotationEncoder);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(getName() + "/DriveTemp", driveMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(getName() + "/RotationTemp", rotationMotor.getDeviceTemp().getValueAsDouble());

    // Refresh cached values in background
    StatusSignal.waitForAll(
        0,
        drivePositionSignal,
        driveVelocitySignal,
        driveCurrentSignal,
        driveVoltageSignal,
        driveTempSignal,
        rotationPositionSignal,
        rotationVelocitySignal,
        rotationCurrentSignal,
        rotationVoltageSignal,
        rotationTempSignal,
        rotationAbsoluteSignal,
        rotationAbsoluteVelSignal);
  }

  @Override
  public void simulationPeriodic() {
    driveSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    rotationSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    rotationEncoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    driveSim.setInput(driveSimState.getMotorVoltage());
    rotationSim.setInput(rotationSimState.getMotorVoltage());

    driveSim.update(0.02);
    rotationSim.update(0.02);

    double driveVel = driveSim.getOutput(0) / DRIVE_METERS_PER_ROTATION;
    driveSimState.setRotorVelocity(driveVel);
    driveSimState.addRotorPosition(driveVel * 0.02);

    double rotationPos = rotationSim.getOutput(0) / ROTATION_DEGREES_PER_ROTATION;
    double rotationDeltaPos = rotationPos - rotationPositionSignal.getValueAsDouble();
    rotationSimState.addRotorPosition(rotationDeltaPos);
    rotationSimState.setRotorVelocity(rotationDeltaPos / 0.02);
    rotationEncoderSimState.setRawPosition(rotationSim.getOutput(0) / 360.0);
    rotationEncoderSimState.setVelocity(getRotationVelocityDegreesPerSecond() / 360.0);
  }

  /**
   * Set the desired state of this module
   *
   * @param desiredState Desired state of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState.optimize(getState().angle);
    this.targetState = desiredState;

    // Don't run the motors if the desired speed is less than 5% of the max
    if (Math.abs(desiredState.speedMetersPerSecond) < DRIVE_MAX_VEL * 0.01) {
      stopMotors();
      return;
    }

    double deltaRot = targetState.angle.getDegrees() - getAbsoluteRotationDegrees();
    if (deltaRot > 180) {
      deltaRot -= 360;
    } else if (deltaRot < -180) {
      deltaRot += 360;
    }
    double targetAngle = getRelativeRotationDegrees() + deltaRot;
    

    StatusCode driveStatus =
        driveMotor.setControl(
            new VelocityVoltage(targetState.speedMetersPerSecond / DRIVE_METERS_PER_ROTATION));
    if (!driveStatus.isOK()) {
      addFault(
          "[Drive Motor]: Status code: "
              + driveStatus.getName()
              + ". "
              + driveStatus.getDescription(),
          driveStatus.isWarning());
    }

    StatusCode rotationStatus =
        rotationMotor.setControl(new PositionVoltage(targetAngle / ROTATION_DEGREES_PER_ROTATION));
    if (!rotationStatus.isOK()) {
      addFault(
          "[Rotation Motor]: Status code: "
              + rotationStatus.getName()
              + ". "
              + rotationStatus.getDescription(),
          rotationStatus.isWarning());
    }
  }

  /** Stop all motors */
  public void stopMotors() {
    // NeutralOut request for coast mode
    driveMotor.setControl(new NeutralOut());
    rotationMotor.setControl(new NeutralOut());
  }

  /**
   * Get the position of the drive motor in meters
   *
   * @return How far this module has driven in meters
   */
  public double getDrivePositionMeters() {
    return drivePositionSignal.getValueAsDouble() * DRIVE_METERS_PER_ROTATION;
  }

  /**
   * Get the relative rotation of this module in degrees.
   *
   * @return Relative rotation
   */
  public double getRelativeRotationDegrees() {
    return rotationPositionSignal.getValueAsDouble() * ROTATION_DEGREES_PER_ROTATION;
  }

  /**
   * Get the absolute rotation of this module in degrees.
   *
   * @return Absolute rotation
   */
  public double getAbsoluteRotationDegrees() {
    return rotationAbsoluteSignal.getValueAsDouble() * 360.0;
  }

  /**
   * Get the velocity of the drive motor in meters/sec
   *
   * @return Drive motor velocity
   */
  public double getDriveVelocityMetersPerSecond() {
    return driveVelocitySignal.getValueAsDouble() * DRIVE_METERS_PER_ROTATION;
  }

  /**
   * Get the velocity of the rotation motor in deg/sec
   *
   * @return Rotation motor velocity
   */
  public double getRotationVelocityDegreesPerSecond() {
    return rotationVelocitySignal.getValueAsDouble() * ROTATION_DEGREES_PER_ROTATION;
  }

  /**
   * Get the rotation of this module as a Rotation2d
   *
   * @return Rotation2d for this module
   */
  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getAbsoluteRotationDegrees());
  }

  /**
   * Get the current state of this module
   *
   * @return Current SwerveModuleState
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getRotation());
  }

  /**
   * Get the current positions of this module
   *
   * @return Current SwerveModulePosition
   */
  public SwerveModulePosition getPositions() {
    return new SwerveModulePosition(getDrivePositionMeters(), getRotation());
  }

  /**
   * Update the rotation offset for this module. This will assume that the current module position
   * should be the new zero.
   */
  public void updateRotationOffset() {
    double currentOffset = rotationEncoderConfig.MagnetSensor.MagnetOffset;
    double offset = (currentOffset - rotationAbsoluteSignal.getValueAsDouble()) % 1.0;
    Preferences.setDouble(getName() + "RotationOffset", offset * 360.0);
    rotationEncoderConfig.MagnetSensor.MagnetOffset = offset;
    rotationEncoder.getConfigurator().apply(rotationEncoderConfig);
    syncRotationEncoders();
  }

  /** Sync the relative rotation encoder (falcon) to the value of the absolute encoder (CANCoder) */
  public void syncRotationEncoders() {
    rotationMotor.setPosition(getAbsoluteRotationDegrees() / ROTATION_DEGREES_PER_ROTATION);
  }

  public void lockModule() {
    double targetAngle = -45;
    if (moduleCode == ModuleCode.FL || moduleCode == ModuleCode.BR) {
      targetAngle = 45;
    }

    targetState = new SwerveModuleState(0, Rotation2d.fromDegrees(targetAngle));
    targetState.optimize(getRotation());

    double deltaRot = targetState.angle.getDegrees() - getAbsoluteRotationDegrees();
    if (deltaRot > 180) {
      deltaRot -= 360;
    } else if (deltaRot < -180) {
      deltaRot += 360;
    }
    double angle = getRelativeRotationDegrees() + deltaRot;


    //MAYBE? .setRotorControl
    driveMotor.setControl(new StaticBrake());
    rotationMotor.setControl(new PositionVoltage(angle / ROTATION_DEGREES_PER_ROTATION));
  }

  public SwerveModuleState getTargetState() {
    return targetState;
  }

  @Override
  public Command systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  clearFaults();
                  driveMotor.stopMotor();
                  rotationMotor.set(0.3);
                },
                this),
            Commands.waitSeconds(0.3),
            Commands.runOnce(
                () -> {
                  if (getRotationVelocityDegreesPerSecond() < 20) {
                    addFault(
                        "[System Check] Rotation motor encoder velocity measured too slow",
                        false,
                        true);
                  }
                  if (rotationAbsoluteVelSignal.getValueAsDouble() * 360 < 20) {
                    addFault(
                        "[System Check] Absolute encoder velocity measured too slow", false, true);
                  }
                },
                this),
            Commands.run(
                    () -> {
                      double deltaRot = 90 - getAbsoluteRotationDegrees();
                      if (deltaRot > 180) {
                        deltaRot -= 360;
                      } else if (deltaRot < -180) {
                        deltaRot += 360;
                      }
                      double angle = getRelativeRotationDegrees() + deltaRot;
                      rotationMotor.setControl(
                          new PositionVoltage(angle / ROTATION_DEGREES_PER_ROTATION));
                    },
                    this)
                .withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (getAbsoluteRotationDegrees() < 70 || getAbsoluteRotationDegrees() > 110) {
                    addFault(
                        "[System Check] Rotation Motor did not reach target position", false, true);
                  }
                },
                this),
            Commands.runOnce(
                () -> {
                  driveMotor.set(0.1);
                  rotationMotor.setControl(new StaticBrake());
                },
                this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(
                () -> {
                  if (getDriveVelocityMetersPerSecond() < 0.25) {
                    addFault("[System Check] Drive motor encoder velocity too slow", false, true);
                  }
                  driveMotor.setControl(new StaticBrake());
                },
                this),
            Commands.waitSeconds(0.25),
            Commands.run(
                    () -> {
                      double deltaRot = 0 - getAbsoluteRotationDegrees();
                      if (deltaRot > 180) {
                        deltaRot -= 360;
                      } else if (deltaRot < -180) {
                        deltaRot += 360;
                      }
                      double angle = getRelativeRotationDegrees() + deltaRot;
                      rotationMotor.setControl(
                          new PositionVoltage(angle / ROTATION_DEGREES_PER_ROTATION));
                    },
                    this)
                .withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (Math.abs(getAbsoluteRotationDegrees()) > 20) {
                    addFault("[System Check] Rotation did not reach target position", false, true);
                  }
                },
                this))
        .until(() -> !getFaults().isEmpty())
        .andThen(Commands.runOnce(this::stopMotors, this));
  }
}
