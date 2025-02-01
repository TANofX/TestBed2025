// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.robot.commands.ElevatorJoystickControl;
import frc.robot.commands.Notifications;
import frc.robot.commands.SwerveDriveWithGamepad;
import frc.robot.subsystems.*;
import frc.robot.util.RobotMechanism;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;



public class RobotContainer {
  // Controllers
  public static final XboxControllerWrapper driver = new XboxControllerWrapper(0, 0.1);
  public static final XboxControllerWrapper coDriver = new XboxControllerWrapper(1, 0.1);

  

  // Subsystems
  public static final Swerve swerve = new Swerve();// new Swerve();
  public static final LEDs LEDs = new LEDs();
  public static final Elevator elevator = new Elevator(Constants.Elevator.motorCanID);
  public static final RobotMechanism robotMechanism = new RobotMechanism();

  // Other Hardware
  public static final PowerDistribution powerDistribution = new PowerDistribution();

  // Vision clients
  // public static final JetsonClient jetson = new JetsonClient();

  public RobotContainer() {

    SmartDashboard.putData(swerve.zeroModulesCommand());
    configureButtonBindings();
    LEDs.setDefaultCommand(new Notifications());
  
  
    // Register Named Commands for pathplanner
    //ADD THESE COMMANDS ONCE WE DEVELOP THEM MORE:
    NamedCommands.registerCommand("ElevatorL4", new getElevatorHeightCommand(0.00000001);
    NamedCommands.registerCommand("ElevatorL1", new getElevatorHeightCommand(0.00000001);
    NamedCommands.registerCommand("ElevatorIntake", new getElevatorHeightCommand(0.00001);
    //NamedCommands.registerCommand("Collect", new ______());
  
    
    //Do I need this?
    elevator.setDefaultCommand(new ElevatorJoystickControl(driver::getLeftY));

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

    
   //ONCE WE ADD ALGAE TO MAIN THESE COMMANDS SHOULD WORK:
   // driver.LT().onTrue(new getAlgaeIntakeCommand());
   // driver.RT().onTrue(new shootAlgaeCommand());
   // driver.START().onTrue(new ); //callibrate elevator

   

   //_________OLD CODE BELOW____________
   /*
     * 
     *     
    driver.LT().onTrue(new SafePosition());
    driver.RB().onTrue(new ClimbPosition());
    driver.LB().onTrue(new ElevatorToMin());
    driver.X().whileTrue(new ReverseIntake());          
    driver.DLeft()
           .onTrue((new ElevateShooter(Constants.Shooter.SHOOT_IN_SPEAKER_AT_SUBWOOFER).alongWith(Commands.runOnce(() -> {
          shooter.startMotorsForShooter(fireControl.getVelocity());
           }, shooter))).andThen(new Shoot(false).andThen(Commands.waitSeconds(.5).andThen(Commands.runOnce(() -> {
           shooter.stopMotors();

           })))));

    
    driver.DRight().onTrue((new ElevateShooter(Constants.Shooter.SHOOT_AT_PODIUM).alongWith(Commands.runOnce(() -> {
      shooter.startMotorsForShooter(fireControl.getVelocity());
   }, shooter))).andThen(new Shoot(false).andThen(Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {
      shooter.stopMotors();
    })))));
    driver.RT().whileTrue(new ConditionalCommand(new IntakeNote(), (new IntakeNote().alongWith(new ReadyToPassNote())).andThen(new TransferNote()), shooterWrist::isStowed));
    
    
    
        //Commands.waitSeconds(.5).andThen(new Shoot().andThen(Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {
          //shooter.stopMotors();
       // }, shooter))))));
   
    //coDriver.X().onTrue(new ElevatorToMin());
    coDriver.RB().onTrue(new ReadyToPassNote().andThen(new TransferNote()));
    coDriver.LB().onTrue(new CalibrateElevator());
    coDriver.DUp().whileTrue(new ExtendElevator());
    coDriver.DDown().whileTrue(new RetractElevator());
    coDriver.LT().onTrue(shootInAmpCommand());
    coDriver.RT().onTrue(shootInSpeaker());
    coDriver.START();
    coDriver.B().toggleOnTrue(new ManualShooterElevation(coDriver::getRightY));
    coDriver.X().onTrue(new CancelShooter());
   
  /*   
        }, shooter))).andThen(new Shoot().andThen(Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {
          shooter.stopMotors();

        }))))); */

    //Commands.waitSeconds(.5).andThen(new Shoot().andThen(Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {
          //shooter.stopMotors();
       // }, shooter))))));
   
    //coDriver.X().onTrue(new ElevatorToMin());
    coDriver.START();
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
    robotMechanism.update();
  }
}