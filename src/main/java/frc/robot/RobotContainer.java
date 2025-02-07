// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.robot.commands.ElevatorJoystickControl;
import frc.robot.commands.CoralHandlerAngleEstimator;
import frc.robot.commands.ManualCoralHandlerVertical;
import frc.robot.commands.Notifications;
import frc.robot.subsystems.*;


public class RobotContainer {
  // Controllers
  public static final XboxControllerWrapper driver = new XboxControllerWrapper(0, 0.1);
  public static final XboxControllerWrapper coDriver = new XboxControllerWrapper(1, 0.1);

  // Subsystems
  public static final Swerve swerve = new Swerve();// new Swerve();
  public static final LEDs LEDs = new LEDs();
  public static final Elevator elevator = new Elevator(Constants.Elevator.motorCanID);
  public static final RobotMechanicalConfiguration robotMechanism = new RobotMechanicalConfiguration();

  // Other Hardware
  public static final PowerDistribution powerDistribution = new PowerDistribution();
  public static final CoralHandler coralHandler = new CoralHandler(Constants.CoralHandler.outtakeMotorID, Constants.CoralHandler.horizontalMotorID, Constants.CoralHandler.verticalMotorID, Constants.CoralHandler.horizontalEncoderID, Constants.CoralHandler.verticalEncoderID);
  // Vision clients
  // public static final JetsonClient jetson = new JetsonClient();

  public RobotContainer() {
    coralHandler.registerSystemCheckWithSmartDashboard();
    SmartDashboard.putData(swerve.zeroModulesCommand());
    configureButtonBindings();
    LEDs.setDefaultCommand(new Notifications());
    elevator.setDefaultCommand(new ElevatorJoystickControl(driver::getLeftY));

    coralHandler.setDefaultCommand(new ManualCoralHandlerVertical(coDriver::getLeftY));
    // SmartDashboard.putData(intake.getIntakePivotTuner());
    // SmartDashboard.putData(intake.getIntakeTuner());
    //SmartDashboard.putData("Tune Elevation", shooterWrist.getElevationTunerCommand());
    //SmartDashboard.putData("Tune Shooter", shooter.getShooterTunerCommand());
    //SmartDashboard.putData("Tune Shooter Intake", shooter.getIntakeTunerCommand());
    //SmartDashboard.putData("Tune Intake", intake.getIntakeTuner());
    // SmartDashboard.putData(Commands.runOnce(() -> {
    // intake.updateRotationOffset();}, intake));

    //SmartDashboard.putData("Tune Elevator Motor", elevator.getHeightTunerCommand());
    //SmartDashboard.putData("Elevator Extents", new FindMotorExtents());

    // SmartDashboard.putData("Robot At Center Blue Ring", Commands.runOnce(() -> {
    //   swerve.resetOdometry(new Pose2d(new Translation2d(2.9, 5.55), Rotation2d.fromDegrees(0)));
    // }, swerve));
    // SmartDashboard.putData("Robot At Red Speaker", new AtRedSubWoofer());

    // Register Named Commands for pathplanner
    //NamedCommands.registerCommand("ReadyToShootInSpeaker", new ShootInSpeaker());
    //NamedCommands.registerCommand("SpeakerShot", new Shoot(false));
    //NamedCommands.registerCommand("New AutoSpeakerShot", newAutoShootInSpeaker());
    // NamedCommands.registerCommand("", );
    
    //PPHolonomicDriveController.setRotationTargetOverride(this::overrideAngle);
  }
  

  private void configureButtonBindings() {
    coDriver.START();
    coDriver.RT().onTrue(new CoralHandlerAngleEstimator());
        //Commands.waitSeconds(.5).andThen(new Shoot().andThen(Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {
          //shooter.stopMotors();
       // }, shooter))))));
   
    //coDriver.X().onTrue(new ElevatorToMin());
    SmartDashboard.putData("Calibrate Elevator", elevator.getCalibrationCommand());
    SmartDashboard.putData("Check Elevator", elevator.getSystemCheckCommand());
    SmartDashboard.putData("Elevator 1.25", elevator.getElevatorHeightCommand(1.25));
    SmartDashboard.putData("Elevator 0.0", elevator.getElevatorHeightCommand(0.0));
  /*   
        }, shooter))).andThen(new Shoot().andThen(Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {
          shooter.stopMotors();

        }))))); */
  
  }


  public static void periodic() {
    robotMechanism.updateMechanism();
  }
}