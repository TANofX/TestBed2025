// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.robot.commands.ManualCoralHandler;
import frc.robot.commands.CoralHandlerAngleEstimator;
import frc.robot.commands.Notifications;
import frc.robot.subsystems.*;
import frc.robot.util.RobotMechanism;



public class RobotContainer {
  // Controllers
  public static final XboxControllerWrapper driver = new XboxControllerWrapper(0, 0.1);
  public static final XboxControllerWrapper coDriver = new XboxControllerWrapper(1, 0.1);

  // Subsystems
  public static final Vision vision = new Vision();
  public static final Swerve swerve = new Swerve();// new Swerve();
  public static final LEDs LEDs = new LEDs();
  public static final Elevator elevator = new Elevator(Constants.Elevator.motorCanID);
  public static final RobotMechanism robotMechanism = new RobotMechanism();

   public static final AlgaeHandler leftAlgaeHandler = new AlgaeHandler(Constants.AlgaeHandler.leftAlgaeMotorCANID, Constants.AlgaeHandler.leftAlgaeSolenoidID,Constants.AlgaeHandler.leftAlgaeHallEffectID,Constants.AlgaeHandler.leftAlgaeLimitID);
   public static final AlgaeHandler rightAlgaeHandler = new AlgaeHandler(Constants.AlgaeHandler.rightAlgaeMotorCANID, Constants.AlgaeHandler.rightAlgaeSolenoidID, Constants.AlgaeHandler.rightAlgaeHallEffectID, Constants.AlgaeHandler.rightAlgaeLimitID);
  // Other Hardware
  public static final PowerDistribution powerDistribution = new PowerDistribution();
  public static final CoralHandler coralHandler = new CoralHandler(Constants.CoralHandler.outtakeMotorID, Constants.CoralHandler.horizontalMotorID, Constants.CoralHandler.verticalMotorID, Constants.CoralHandler.horizontalEncoderID, Constants.CoralHandler.verticalEncoderID);
  public static final Climber climber = new Climber(Constants.Climber.MOTOR_CANID, Constants.Climber.PCMID, Constants.Climber.SOLONOIDID, Constants.Climber.climberEncoderCanID);
  // Vision clients
  // public static final JetsonClient jetson = new JetsonClient();

  public RobotContainer() {
    coralHandler.registerSystemCheckWithSmartDashboard();
    SmartDashboard.putData("Elevator Test", elevator.getSystemCheckCommand());

    SmartDashboard.putData(swerve.zeroModulesCommand());
    configureButtonBindings();
    LEDs.setDefaultCommand(new Notifications());
    //elevator.setDefaultCommand(new ElevatorJoystickControl(coDriver::getLeftY));
    SmartDashboard.putData("Left Algae Handler Test", leftAlgaeHandler.getSystemCheckCommand());
    SmartDashboard.putData("Right Algae Handler Test", rightAlgaeHandler.getSystemCheckCommand());
  }
  
  private void configureButtonBindings() {
    driver.A().onTrue(elevator.getElevatorHeightCommand(Constants.Elevator.MIN_HEIGHT_METERS));
    driver.B().onTrue(elevator.getElevatorHeightCommand(Units.inchesToMeters(20.0)));
    driver.Y().onTrue(elevator.getElevatorHeightCommand(Units.inchesToMeters(40.0)));
    driver.X().onTrue(elevator.getElevatorHeightCommand(Constants.Elevator.MAX_HEIGHT_METERS));
    
    coDriver.START();
    coDriver.RT().onTrue(new CoralHandlerAngleEstimator());
    // coralHandler.setDefaultCommand(new ManualCoralHandler(coDriver::getLeftY, coDriver::getLeftX));
    coralHandler.setDefaultCommand(new ManualCoralHandler(() -> {
      if (coDriver.DUp().getAsBoolean()) {
        return 0.5;
      }
      if (coDriver.DDown().getAsBoolean()){
        return -0.5;
      }
      return 0.0;
    }, () -> {
      if (coDriver.DRight().getAsBoolean()) {
        return -0.5;
      }
      if (coDriver.DLeft().getAsBoolean()) {
        return 0.5;
      }
      return 0.0;
    }));

    SmartDashboard.putData("Calibrate Elevator", elevator.getCalibrationCommand());
    SmartDashboard.putData("Check Elevator", elevator.getSystemCheckCommand());
    SmartDashboard.putData("Elevator 1.25", elevator.getElevatorHeightCommand(1.25));
    SmartDashboard.putData("Elevator 0.0", elevator.getElevatorHeightCommand(0.0));
  }


  public static void periodic() {
    robotMechanism.update();
  }
}