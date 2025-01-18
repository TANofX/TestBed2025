// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.nio.file.OpenOption;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.math.MathUtil;
import frc.lib.pid.TuneVelocitySparkPIDController;



import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;



public class AlgaeHandler extends AdvancedSubsystem {
private SparkFlex algaeMotor;
private Solenoid algaePiston;
private SparkLimitSwitch algaeLimitSwitch;
private double speedInRPM;
private  SparkClosedLoopController algaeMotorController;
private DigitalInput algaeHallEffect;


  
  /** Creates a new AlgaeHandler. */
  public AlgaeHandler(int algaeMotorCANID, int algaeSolenoidID, int algaeHallEffectID) {
    //creating motor/solenoid/switches/controllers
    algaeMotor = new SparkFlex(Constants.AlgaeHandler.algaeMotorCanID, MotorType.kBrushed);
    algaePiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.AlgaeHandler.algaeSolenoidID);
    algaeLimitSwitch = algaeMotor.getForwardLimitSwitch();
    algaeMotorController = algaeMotor.getClosedLoopController();
    algaeHallEffect = new DigitalInput(Constants.AlgaeHandler.algaeHallEffectID);


    SparkFlexConfig algaeMotorConfig = new SparkFlexConfig();
    ClosedLoopConfig algaeMotorPIDConfig = algaeMotorConfig.closedLoop;
    CANcoderConfiguration algaeEncoderConfig = new CANcoderConfiguration();
    algaeMotorConfig.idleMode(IdleMode.kBrake);
    algaeMotorPIDConfig.pid(Constants.AlgaeHandler.algaeMotorP, Constants.AlgaeHandler.algaeMotorI, Constants.AlgaeHandler.algaeMotorD);
    algaeMotorPIDConfig.velocityFF(Constants.AlgaeHandler.algaeFF);
    algaeMotorPIDConfig.iZone(Constants.AlgaeHandler.algaeIZone);
    algaeMotorPIDConfig.maxMotion.maxVelocity(Constants.AlgaeHandler.algaeMotorMaxVelocity);
    algaeMotorPIDConfig.maxMotion.maxAcceleration(Constants.AlgaeHandler.algaeMotorMaxAcceleration);
    algaeMotorPIDConfig.maxMotion.allowedClosedLoopError(Constants.AlgaeHandler.algaeMotorAllowedError);
  
    registerHardware("Algae Motor", algaeMotor);
    
    
  }

public void lowerAlgaeIntake() {
  algaePiston.set(true);
}

public void raiseAlgaeIntake() {
  algaePiston.set(false);
}

public void stopAlgaeMotor() {
  algaeMotor.stopMotor();
}

public void runAlgaeMotor() {
  //velocity value is a place holder because I didnt know what I was doing :D
  algaeMotorController.setReference(.4, ControlType.kVelocity);
}

public void reverseAlgaeMotor() {
  //velocity value is a place holder :D
  algaeMotorController.setReference(-.4, ControlType.kVelocity);
}

public void limitSwitchTrigger() {
  algaeLimitSwitch.isPressed();

}
public void hallEffectTrigger() {
  algaeHallEffect.close();
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected Command systemCheckCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'systemCheckCommand'");
  }
}
