package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.io.IOException;

public final class Constants {
  public static final String canivoreBusName = "rio";
  public static final AprilTagFieldLayout apriltagLayout;

  static {
    try {
      apriltagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
      apriltagLayout.getFieldLength();
      apriltagLayout.getFieldWidth();
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static final Translation2d fieldSize = new Translation2d(16.54, 8.02);
  public static final double noteTransferMetersPerSecond = 0.5;

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

  public static final class AutoBalance {
    // public static final PIDConstants BALANCE_CONSTANTS = new PIDConstants(0.3,
    // 0.0, 0.1);
    public static final double maxVelAuto = 0.4;
    public static final double maxVelTele = 0.3;
  }

  public static final class FireControl {
    public static final double FINAL_Y_VELOCITY = 3;
    public static final double ACCELERATION = 9.81;
    public static final double HEIGHT = Units.inchesToMeters(72);
    public static final double TARGET_VELOCITY_MPS = 15;
    // public static final double SHOOTER_HEIGHT = 24;
    // public static final double HEIGHT = SPEAKER_HEIGHT - SHOOTER_HEIGHT;
    public static final Pose2d BLUE_SPEAKER_POSITION = new Pose2d(Units.inchesToMeters(-1.5),
        Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0.0));
    public static final Pose2d RED_SPEAKER_POSITION = new Pose2d(Units.inchesToMeters(652.73),
        Units.inchesToMeters(218.42), Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_AMP_CORNER_POSITION = Constants.apriltagLayout.getTagPose(6).get().toPose2d();
    public static final Pose2d RED_AMP_CORNER_POSITION = Constants.apriltagLayout.getTagPose(5).get().toPose2d();
    // public static final Pose2d RED_SPEAKER_POSITION = new Pose2d(8.3,4.1,
    // Rotation2d.fromDegrees(0));
    public static final Transform2d SHOOTER_OFFSET = new Transform2d(Units.inchesToMeters(-6), Units.inchesToMeters(0),
        Rotation2d.fromDegrees(180));
    public static final Rotation2d AZMUTH_OFFSET = Rotation2d.fromDegrees(5.0);
    public static final Translation2d FEEDOFFSET = new Translation2d(0.0, 1.5);
  }

  public static final class Elevator {
    public static final int ELEVATORMOTOR_ID = 30;
    public static final double METERS_PER_REV = .180;
    public static final int MOTOR_REV_PER_ROTATION = 100;
    public static final double METERS_PER_MOTOR_REV = METERS_PER_REV / MOTOR_REV_PER_ROTATION;
    public static final double MAX_HEIGHT = -89.0;
    public static final double MIN_HEIGHT = 0.15;
    public static final double elevatorMotorP = 0.00005;
    public static final double elevatorMotorI = 0.00;
    public static final double elevatorMotorD = 0.00;
    public static final double elevatorMotorIZone = 5;
    public static final double elevatorMotorFeedForward = 0.0001;
    public static final double elevatorMotorMinVelocity = 0;
    public static final double elevatorMotorMaxVelocity = 5500;
    public static final double elevatorMotorMaxAcceleration = 10000;
    public static final double elevatorMotorClosedLoppError = 2;
  }

  public static final class CoralHandler {
    // TODO figure out actual IDs
    public static final double MeterPerMotorRevolution = 0.0;

    public static final double coralEndEffectorLength = 4.0;
    public static final double coralEndEffectorMass = 8.5;
    public static final double outtakeWheelRadius = 2.0;

    public static final int outtakeMotorID = 1;
    public static final int horizontalMotorID = 2;
    public static final int horizontalEncoderID = 3;
    public static final int verticalMotorID = 4;
    public static final int verticalEncoderID = 5;

    public static final double outtakeMotorGearing = 1;
    public static final double horizontalMotorGearing = 1.0 / 100.0;
    public static final double verticalMotorGearing = 1.0/100.0;

    public static final double outtakeJKgMetersSquared = (.5 * coralEndEffectorMass * Math.pow(outtakeWheelRadius, 2));
    public static final double outtakeMotorMinVelocity = 0.0;
    // public static final int outtakeEncoderID = 0.0;
    

    public static final double horizontalMotorP = 0.0;
    public static final double verticalMotorP = 0.0;
    public static final double horizontalMotorI = 0.0;
    public static final double verticalMotorI = 0.0;
    public static final double horizontalMotorD = 0.0;
    public static final double verticalMotorD = 0.0;
    public static final double horizontalMotorFeedForward = 0.0;
    public static final double verticalMotorFeedForward = 0.0;
    public static final double horizontalMotorIZone = 0.0;
    public static final double verticalMotorIZone = 0.0;
    public static final double horizontalMotorMaxAccleration = 0.0;
    public static final double verticalMotorMaxAccleration = 0.0;
    public static final double horizontalMotorMaxVelocity = 0.0;
    public static final double verticalMotorMaxVelocity = 0.0;
    public static final double horizontalMotorClosedLoopError = 0.0;
    public static final double verticalMotorClosedLoopError = 0.0;
    public static final double horizontalJKgMetersSquared = 1.0/3.0 * coralEndEffectorMass * Math.pow(coralEndEffectorLength, 2.0);
    public static final double verticalJKgMetersSquared = 1.0/3.0 * coralEndEffectorMass * Math.pow(coralEndEffectorLength, 2.0);
    public static final double horizontalMinAngleInRadians = Units.degreesToRadians(-100.0);
    public static final double verticalMinAngleInRadians = Units.degreesToRadians(-90.0);
    public static final double horizontalMaxAngleInRadians = Units.degreesToRadians(100.0);
    public static final double verticalMaxAngleInRadians = Units.degreesToRadians(90.0);
    public static final double horizontalStartingAngleInRadians = 0.0;
    public static final double verticalStartingAngleInRadians = Units.degreesToRadians(90);
    public static final double horizontalMotorStdDev = 0.0;
    public static final double verticalMotorStdDev = 0.0;
    public static final double horizontalRotationDegreesPerRotation = 360.0 / horizontalMotorGearing;
    public static final double verticalRotationDegreesPerRotation = 360.0 / verticalMotorGearing;
    public static final double horizontalMotorMinVelocity = 0.0;
    public static final double verticalMotorMinVelocity = 0.0;
    // ???? what do you do for offset for position (use gear ratio to figure out)
    // public static final double RotationDegreesPerRotation = 0;
  }
}
