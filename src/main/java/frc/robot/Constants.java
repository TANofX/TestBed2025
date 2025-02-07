package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import java.io.IOException;

public final class Constants {
  public static final String canivoreBusName = "rio";
  public static final AprilTagFieldLayout apriltagLayout;
  public static final Translation2d fieldSize;

  static {
    try {
      apriltagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
      apriltagLayout.getFieldLength();
      apriltagLayout.getFieldWidth();
      fieldSize = new Translation2d(apriltagLayout.getFieldLength(), apriltagLayout.getFieldWidth());
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static final class Elevator {
    public static final int motorCanID = 1;

    public static final double P = 0.05;
    public static final double I = 0.00;
    public static final double D = 0.00;
    public static final double FF = 0.00015;

    public static final double METERS_PER_MOTOR_REVOLUTION = Units.inchesToMeters(1.0 / 4.0);
    public static final double ELEVATOR_MASS = Units.lbsToKilograms(20.0);
    public static final double GEAR_RATIO = 1.0;
    public static final double MIN_HEIGHT_METERS = 0.0;
    public static final double MAX_HEIGHT_METERS = Units.inchesToMeters(84.0);
    public static final double STARTING_HEIGHT_METERS = MIN_HEIGHT_METERS + (MIN_HEIGHT_METERS + MAX_HEIGHT_METERS) / 2.0;

    public static final double MAX_ACCELERATION = 10000.0;
    public static final double MAX_VELOCITY = 5000.0;
  };

  public static final class LEDs {
    public static final int stripPwm = 0;
    public static final int stripLength = 150;
  }

  public static final class Swerve {
    public static final int imuCanID = 3;
    public static final double maxVelTele = 4.7;
    public static final double maxAccelTele = 6.0;
    public static final double maxAngularVelTele = Units.degreesToRadians(180);
    public static final double maxAngularAccelTele = Units.degreesToRadians(540);
    public static final double teleAngleHoldFactor = 3.0;

    public static final double SPEAKER_CONTROLLER_kP = 1.0;
    public static final double SPEAKER_CONTROLLER_kI = 0.0;
    public static final double SPEAKER_CONTROLLER_kD = 0.01;

    public static final class Odometry {
      public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.05);
      public static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    }

    public static final class PathFollowing {
      // public static final PIDConstants TRANSLATION_CONSTANTS = new
      // PIDConstants(5.0, 0.0, 0.0);
      // public static final PIDConstants ROTATION_CONSTANTS = new PIDConstants(2.0,
      // 0.0, 0.0);
    }

    public static final class FrontLeftModule {
      public static final int driveMotorCanID = 7;
      public static final int rotationMotorCanID = 8;
      public static final int rotationEncoderCanID = 13;
      public static Translation2d moduleOffset = new Translation2d(Units.inchesToMeters(11.25),
          Units.inchesToMeters(12.25));
    }

    public static final class FrontRightModule {
      public static final int driveMotorCanID = 10;
      public static final int rotationMotorCanID = 11;
      public static final int rotationEncoderCanID = 12;
      public static Translation2d moduleOffset = new Translation2d(Units.inchesToMeters(11.25),
          -Units.inchesToMeters(12.25));
    }

    public static final class BackLeftModule {
      public static final int driveMotorCanID = 4;
      public static final int rotationMotorCanID = 5;
      public static final int rotationEncoderCanID = 14;
      public static Translation2d moduleOffset = new Translation2d(-Units.inchesToMeters(11.25),
          Units.inchesToMeters(12.25));
    }

    public static final class BackRightModule {
      public static final int driveMotorCanID = 6;
      public static final int rotationMotorCanID = 9;
      public static final int rotationEncoderCanID = 15;
      public static Translation2d moduleOffset = new Translation2d(-Units.inchesToMeters(11.25),
          -Units.inchesToMeters(12.25));
    }
  }

  public static final class CoralHandler {
    // TODO figure out actual IDs
    public static final double MeterPerMotorRevolution = 0.0;

    public static final int outtakeMotorID = 1;
    public static final int horizontalMotorID = 2;
    public static final int horizontalEncoderID = 3;
    public static final int verticalMotorID = 4;
    public static final int verticalEncoderID = 5;

    // !! `coralEndEffectorLength` is IN METERS
    public static final double coralEndEffectorLength = 0.25;
    public static final double coralEndEffectorMass = 0.5;

    public static final double outtakeWheelMass = Units.lbsToKilograms(0.5);
    public static final double outtakeWheelRadius = 0.02; // TODO what is the actual outtake wheel radius
    
    public static final double coralIntakeSpeed = 0;
    public static final double coralOuttakeSpeed = 0;
    
    public static final double outtakeMotorGearing = 1.0;
    public static final double horizontalGearRatio = 100.0;
    public static final double verticalGearRatio = 100.0;

    public static final double outtakeJKgMetersSquared = (.5 * outtakeWheelMass * Math.pow(outtakeWheelRadius, 2));
    public static final double outtakeMotorMinVelocity = 0.0;
    // public static final int outtakeEncoderID = 0.0;
    
    public static final double horizontalMotorP = 0.001;
    public static final double horizontalMotorI = 0.0;
    public static final double horizontalMotorD = 0.001;
    public static final double horizontalMotorFeedForward = 1.0 / (565.0*12.0);
    public static final double horizontalMotorIZone = 0.0;
    public static final double horizontalMotorMaxAccleration = 1000.0;
    public static final double horizontalMotorMaxVelocity = 6000.0;
    public static final double horizontalMotorClosedLoopError = 1.0;

    public static final double verticalMotorP = 0.001;
    public static final double verticalMotorI = 0.0;
    public static final double verticalMotorD = 0.001;
    public static final double verticalMotorFeedForward = 1.0 / (565.0*12.0);
    public static final double verticalMotorIZone = 0.0;
    public static final double verticalMotorMaxAccleration = 1.0;
    public static final double verticalMotorMaxVelocity = 6000.0;
    public static final double verticalMotorClosedLoopError = 1.0;
    
    public static final double horizontalJKgMetersSquared = 1.0/3.0 * coralEndEffectorMass * Math.pow(coralEndEffectorLength, 2.0);
    public static final double verticalJKgMetersSquared = 1.0/3.0 * coralEndEffectorMass * Math.pow(coralEndEffectorLength, 2.0);
    
    public static final Rotation2d horizontalMinAngle = Rotation2d.fromDegrees(-100);
    public static final Rotation2d horizontalMaxAngle = Rotation2d.fromDegrees(100);

    public static final Rotation2d verticalMinAngle = Rotation2d.fromDegrees(-90);
    public static final Rotation2d verticalMaxAngle = Rotation2d.fromDegrees(90);
    
    public static final double horizontalStartingAngleInRadians = 0.0;
    public static final double verticalStartingAngleInRadians = 0.0;
    
    public static final double horizontalMotorStdDev = 0.0;
    public static final double verticalMotorStdDev = 0.0;
    
    public static final double horizontalMotorMinVelocity = 0.0;
    public static final double verticalMotorMinVelocity = 0.0;

    public static final double verticalRotationDegreesPerRotation = 360 / verticalGearRatio;
    public static final double horizontalRotationDegreesPerRotation = 360 / horizontalGearRatio;
    
    //Need different name, for manual coral joystick control
    public static final double verticalAngleChangeDegreesPerSecond = verticalMotorMaxVelocity * verticalGearRatio / 60;
    
    //TODO change degree numbers
    public static final Rotation2d horizontalIntakeAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d horizontalLevel1AngleRight = Rotation2d.fromDegrees(0);
    public static final Rotation2d horizontalLevel1AngleLeft = Rotation2d.fromDegrees(0);
    public static final Rotation2d horizontalLevel2Angle = Rotation2d.fromDegrees(0);
    public static final Rotation2d horizontalLevel3Angle = Rotation2d.fromDegrees(0);
    public static final Rotation2d horizontalLevel4Angle = Rotation2d.fromDegrees(0);
    public static final Rotation2d vertialIntakeAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d vertialLevel1Angle = Rotation2d.fromDegrees(0);
    public static final Rotation2d verticallLevel2Angle = Rotation2d.fromDegrees(0);
    public static final Rotation2d verticallLevel3Angle = Rotation2d.fromDegrees(0);
    public static final Rotation2d verticallLevel4Angle = Rotation2d.fromDegrees(0);
    // ???? what do you do for offset for position (use gear ratio to figure out)
    // public static final double RotationDegreesPerRotation = 0;
  }
}
