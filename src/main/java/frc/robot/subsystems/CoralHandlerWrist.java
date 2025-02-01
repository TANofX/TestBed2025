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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;

public class CoralHandlerWrist extends AdvancedSubsystem {

    private final String name;
    private final double gearRatio;
    private final double clampMin;
    private final double clampMax;
    private final double minVelocity;

    private final SparkMax motor;
    private final SparkClosedLoopController controller;
    private final CANcoder absoluteEncoder;
    private final RelativeEncoder relativeEncoder;
    private final StatusSignal<Angle> absoluteSignal;

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
            double clampMin, // min rotation of the arm in degrees
            double clampMax // max rotation of the arm in degrees
    ) {
        super("CoralHandlerWrist" + name);
        this.name = name;
        this.gearRatio = gearRatio;
        this.minVelocity = minVelocity;
        this.clampMin = clampMin;
        this.clampMax = clampMax;

        this.motor = new SparkMax(motorId, SparkLowLevel.MotorType.kBrushless);
        this.controller = motor.getClosedLoopController();
        this.relativeEncoder = motor.getEncoder();

        LimitSwitchConfig limitConfig = new LimitSwitchConfig();
        limitConfig.forwardLimitSwitchType(limitSwitchType);
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig.forwardSoftLimitEnabled(false);
        softLimitConfig.reverseSoftLimitEnabled(false);
        //softLimitConfig.reverseSoftLimit(clampMin / rotationDegreesPerRotation);
        //softLimitConfig.forwardSoftLimit(clampMax / rotationDegreesPerRotation);

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
        motor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // configure encoder
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = Preferences.getDouble(name.toLowerCase() + "RotationalOffset", 0);
        this.absoluteEncoder = new CANcoder(encoderId);
        absoluteEncoder.getConfigurator().apply(encoderConfig);
        absoluteEncoder.getConfigurator().setPosition(0);
        this.absoluteSignal = absoluteEncoder.getAbsolutePosition();
        absoluteSignal.refresh();

        this.sim = new SparkMaxSim(motor, DCMotor.getNeo550(1));

        registerHardware("Coral " + name + " Motor", motor);
        registerHardware("Coral " + name + " Encoder", absoluteEncoder);

        this.encoderSim = new SparkAbsoluteEncoderSim(motor);
    }

    /**
     * Sets the angle.
     *
     * @param targetAngle                The needed angle to be obtained.
     */
    public void setAngle(Rotation2d targetAngle) {
        //the target rotation of the arm in degrees
        double target = MathUtil.clamp(targetAngle.getDegrees(), clampMin, clampMax);
        //how much is wanted to move in degrees
        double adjustedAngle = absoluteEncoder.getPosition().getValue().in(Degrees) - target; // TODO This is supposed to return double bc it was degrees...?
        //how far to rotate the motor in degrees from where we are currently
        double angleOffset = adjustedAngle * gearRatio;
        //the actual target angle --> the target absolute rotation of the motor
        double neededAngle = Units.rotationsToDegrees(motor.getEncoder().getPosition()) + angleOffset;
        double motorRotations = neededAngle / 360.0;
        controller.setReference(motorRotations, SparkBase.ControlType.kMAXMotionPositionControl); // TODO Is it supposed be position control?
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    public void refreshAbsoluteSignal() {
        absoluteSignal.refresh();
    }

    public Command setAngleCommand(Rotation2d targetAngle) {
        SmartDashboard.putNumber(name + "TargetAngle in Degrees", targetAngle.getDegrees());
        return Commands.sequence(
                Commands.runOnce(() -> setAngle(targetAngle), this),
                Commands.waitUntil(()->
                        Math.abs(targetAngle.getDegrees() - absoluteEncoder.getPosition().getValue().in(Degrees)) < 2
                ),
                Commands.runOnce(motor::stopMotor, this)
        );
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
                            setAngle(Rotation2d.fromDegrees(10)); // TODO Is this a way to set the target angle, because it is a rotation2d?
                        }, this),
                Commands.waitSeconds(5.0),
                Commands.runOnce(
                        () -> {
                            if ((relativeEncoder.getVelocity()) < minVelocity) {
                                addFault("[System Check] " + name + " Coral Motor too slow (forward direction)", false, true);
                            }
                        }, this),
                Commands.runOnce(
                        () -> {
                            setAngle(Rotation2d.fromDegrees(0)); // TODO Is this a way to set the target angle, because it is a rotation2d?
                        }, this),
                Commands.waitSeconds(5.0),
                Commands.runOnce(
                        () -> {
                            if ((relativeEncoder.getVelocity()) < -minVelocity) {
                                addFault("[System Check] " + name + " Coral Motor too slow (backwards direction)", false, true);
                            }
                            motor.stopMotor();
                        }, this));
    }
}
