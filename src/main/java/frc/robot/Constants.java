package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

public final class Constants {
  public static final String canivoreBusName = "rio";
  public static final AprilTagFieldLayout apriltagLayout;
  public static final Translation2d fieldSize;

  static {
    try {
      apriltagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
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
    

    //Class to access the coordinates of the coral on the field.
    public static final class CoralPlacement {
      public static ArrayList<Pose2d> cordinatesCoralRed = new ArrayList<Pose2d>();
      static {
      //ordered in line from A-L
      //rotation degree part of Pos2D is the direction the robot has to face to be flush against the reef for that branch
      cordinatesCoralRed.add(new Pose2d(544.87, 152.03, Rotation2d.fromDegrees(180))); 
      cordinatesCoralRed.add(new Pose2d(544.87, 164.97,  Rotation2d.fromDegrees(180))); 
      cordinatesCoralRed.add(new Pose2d(535.08, 181.89,  Rotation2d.fromDegrees(240)));
      cordinatesCoralRed.add(new Pose2d(523.90, 188.32,  Rotation2d.fromDegrees(240)));
      cordinatesCoralRed.add(new Pose2d(504.39, 188.32,  Rotation2d.fromDegrees(300))); 
      cordinatesCoralRed.add(new Pose2d(493.16, 181.89,  Rotation2d.fromDegrees(300)));
      cordinatesCoralRed.add(new Pose2d(483.44, 164.97,  Rotation2d.fromDegrees(0.0))); 
      cordinatesCoralRed.add(new Pose2d(483.44, 152.03,  Rotation2d.fromDegrees(0.0))); 
      cordinatesCoralRed.add(new Pose2d(493.16, 135.15,  Rotation2d.fromDegrees(60))); 
      cordinatesCoralRed.add(new Pose2d(504.39, 128.65,  Rotation2d.fromDegrees(60))); 
      cordinatesCoralRed.add(new Pose2d(523.90, 128.65,  Rotation2d.fromDegrees(120))); 
      cordinatesCoralRed.add(new Pose2d(535.08, 135.15,  Rotation2d.fromDegrees(120)));
      }

      public static ArrayList<Pose2d> cordinatesCoralBlue = new ArrayList<Pose2d>();
      static {
      //ordered in line from A-L, even though this is "opposite" of blue
      cordinatesCoralBlue.add(new Pose2d(146.052, 164.97, Rotation2d.fromDegrees(0.0)));
      cordinatesCoralBlue.add(new Pose2d(146.052, 152.03,  Rotation2d.fromDegrees(0.0))); 
      cordinatesCoralBlue.add(new Pose2d(155.43, 135.15, Rotation2d.fromDegrees(60))); 
      cordinatesCoralBlue.add(new Pose2d(166.65, 128.65, Rotation2d.fromDegrees(60))); 
      cordinatesCoralBlue.add(new Pose2d(136.51, 128.65, Rotation2d.fromDegrees(120))); 
      cordinatesCoralBlue.add(new Pose2d(197.69, 135.15, Rotation2d.fromDegrees(120))); 
      cordinatesCoralBlue.add(new Pose2d(207.48, 152.03, Rotation2d.fromDegrees(180)));
      cordinatesCoralBlue.add(new Pose2d(207.48, 164.97, Rotation2d.fromDegrees(180))); 
      cordinatesCoralBlue.add(new Pose2d(197.69, 181.89, Rotation2d.fromDegrees(240))); 
      cordinatesCoralBlue.add(new Pose2d(186.51, 188.32, Rotation2d.fromDegrees(240)));
      cordinatesCoralBlue.add(new Pose2d(166.65, 188.32, Rotation2d.fromDegrees(300)));
      cordinatesCoralBlue.add(new Pose2d(155.43, 181.89, Rotation2d.fromDegrees(300))); 
      }
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
}
