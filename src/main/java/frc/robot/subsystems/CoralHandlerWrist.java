package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Degrees;

import java.io.Console;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class CoralHandlerWrist extends AdvancedSubsystem {

    private final String name;
    private final double gearRatio;
    private final Rotation2d armMinRotation;
    private final Rotation2d armMaxRotation;
    private final double minVelocity;

    private final SparkMax motor;
    private final SparkClosedLoopController controller;
    private final CANcoder absoluteEncoder;
    private final RelativeEncoder relativeEncoder;
    private final StatusSignal<Angle> absoluteSignal;

    private final SingleJointedArmSim coralHandlerPhysicsSim;

    // TODO: make these private and expose via methods
    public final SparkAbsoluteEncoderSim encoderSim;
    public final SparkMaxSim sim;

    public CoralHandlerWrist(
            String name, // "Horizontal" or "Vertical"
            int motorId,
            int encoderId,
            double gearRatio,
            double p,
            double i,
            double d,
            double ff,
            double izone,
            double minVelocity,
            double maxVelocity,
            double maxAcceleration,
            double allowedError,
            LimitSwitchConfig.Type limitSwitchType,
            Rotation2d armMinRotation, // min rotation of the arm
            Rotation2d armMaxRotation, // max rotation of the arm
            double jKgMetersSquared,
            double coralEndEffectorLength,
            double startingAngle) {
        super("CoralHandlerWrist" + name);
        this.name = name;
        this.gearRatio = gearRatio;
        this.minVelocity = minVelocity;
        this.armMinRotation = armMinRotation;
        this.armMaxRotation = armMaxRotation;

        this.motor = new SparkMax(motorId, SparkLowLevel.MotorType.kBrushless);
        this.controller = motor.getClosedLoopController();
        this.relativeEncoder = motor.getEncoder();

        LimitSwitchConfig limitConfig = new LimitSwitchConfig();
        limitConfig.forwardLimitSwitchType(limitSwitchType);
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig.forwardSoftLimitEnabled(false);
        softLimitConfig.reverseSoftLimitEnabled(false);
        // softLimitConfig.reverseSoftLimit(clampMin / rotationDegreesPerRotation);
        // softLimitConfig.forwardSoftLimit(clampMax / rotationDegreesPerRotation);

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.inverted(false);
        motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        // configure motor
        ClosedLoopConfig pidConfig = motorConfig.closedLoop;
        pidConfig.pidf(p, i, d, ff);
        pidConfig.iZone((izone));
        pidConfig.maxMotion.maxAcceleration(maxAcceleration);
        pidConfig.maxMotion.maxVelocity(maxVelocity);
        pidConfig.maxMotion.allowedClosedLoopError(allowedError);
        motorConfig.apply(pidConfig);
        motor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

        // configure encoder
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = Preferences.getDouble(name.toLowerCase() + "RotationalOffset", 0);
        this.absoluteEncoder = new CANcoder(encoderId);
        absoluteEncoder.getConfigurator().apply(encoderConfig);
        absoluteEncoder.getConfigurator().setPosition(0);
        this.absoluteSignal = absoluteEncoder.getAbsolutePosition();
        absoluteSignal.refresh();

        coralHandlerPhysicsSim = new SingleJointedArmSim(DCMotor.getNeo550(1),
                gearRatio, jKgMetersSquared, coralEndEffectorLength, armMinRotation.getRadians(),
                armMaxRotation.getRadians(), true,
                startingAngle); // ,Constants.CoralHandler.horizontalMotorStdDev);

        this.sim = new SparkMaxSim(motor, DCMotor.getNeo550(1));

        registerHardware("Coral " + name + " Motor", motor);
        registerHardware("Coral " + name + " Encoder", absoluteEncoder);

        this.encoderSim = new SparkAbsoluteEncoderSim(motor);

    }

    public void simulationPeriodic() {
        double inputVoltage = sim.getAppliedOutput() * RobotController.getBatteryVoltage();

        coralHandlerPhysicsSim.setInput(inputVoltage);

        coralHandlerPhysicsSim.update(0.02);

        double motorVelocity = ((coralHandlerPhysicsSim.getVelocityRadPerSec() / (2 * Math.PI)) * 60)
                * Constants.CoralHandler.horizontalGearRatio;

        sim.iterate(motorVelocity, RobotController.getBatteryVoltage(), 0.02);

        encoderSim.iterate((motorVelocity / gearRatio), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getMotorCurrent()));
    }

    /**
     * Sets the angle.
     *
     * @param targetAngle The needed angle to be obtained.
     */
    public void setAngle(Rotation2d targetAngle) {
        // the target rotation of the arm
        Rotation2d target = Rotation2d.fromDegrees(
                MathUtil.clamp(targetAngle.getDegrees(), armMinRotation.getDegrees(), armMaxRotation.getDegrees()));
        // how much is wanted to move in degrees
        Rotation2d adjustedAngle = new Rotation2d(absoluteEncoder.getPosition().getValue()).minus(target);
        // how far to rotate the motor in degrees from where we are currently
        Rotation2d angleOffset = adjustedAngle.times(gearRatio);
        // the actual target angle --> the target absolute rotation of the motor
        Rotation2d neededAngle = Rotation2d.fromRotations(motor.getEncoder().getPosition()).plus(angleOffset);
        controller.setReference(neededAngle.getRotations(), SparkBase.ControlType.kMAXMotionPositionControl);
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    public void refreshAbsoluteSignal() {
        absoluteSignal.refresh();
    }

    public Command setAngleCommand(Rotation2d targetAngle) {
        SmartDashboard.putNumber(name + "Target Angle in Degrees", targetAngle.getDegrees());
        return Commands.sequence(
                Commands.runOnce(() -> setAngle(targetAngle), this),
                Commands.waitUntil(() -> Math
                        .abs(targetAngle.getDegrees() - absoluteEncoder.getPosition().getValue().in(Degrees)) < 2),
                Commands.runOnce(motor::stopMotor, this));
    }

    public void putAbsoluteEncoderSimAngle() {
        SmartDashboard.putNumber(name + " Absolute Encoder Sim Angle",
                sim.getAbsoluteEncoderSim().getPosition());
    }

    @Override
    public Command systemCheckCommand() {
        return Commands.sequence(
                Commands.runOnce(
                        () -> {
                            setAngle(Rotation2d.fromDegrees(10)); // TODO Is this a way to set the target angle, because
                                                                  // it is a rotation2d?
                        }, this),
                Commands.waitSeconds(5.0),
                Commands.runOnce(
                        () -> {
                            if ((relativeEncoder.getVelocity()) < minVelocity) {
                                addFault("[System Check] " + name + " Coral Motor too slow (forward direction)", false,
                                        true);
                            }
                        }, this),
                Commands.runOnce(
                        () -> {
                            setAngle(Rotation2d.fromDegrees(0)); // TODO Is this a way to set the target angle, because
                                                                 // it is a rotation2d?
                        }, this),
                Commands.waitSeconds(5.0),
                Commands.runOnce(
                        () -> {
                            if ((relativeEncoder.getVelocity()) < -minVelocity) {
                                addFault("[System Check] " + name + " Coral Motor too slow (backwards direction)",
                                        false, true);
                            }
                            motor.stopMotor();
                        }, this));
    }
}
