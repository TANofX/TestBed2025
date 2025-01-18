// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystem.AdvancedSubsystem;

public class Climber extends AdvancedSubsystem {
  private final SparkFlex climbermotor;
  private final Solenoid climberpiston; 
  private final SparkClosedLoopController climbercontroller;

  /** Creates a new Climber. */
  public Climber(int motor_canid, int pcmid, int solonoidid) {
    climberpiston = new Solenoid(PneumaticsModuleType.REVPH, solonoidid);
    climbermotor = new SparkFlex(motor_canid, MotorType.kBrushless);
    climbermotor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters)
    climbercontroller =  climbermotor.getClosedLoopController();
  }


  //HEY I ALEADY PUT IN THE STAGE GEAR RATIOS IN THE CONSTANTS!! -- Shirley C. :)


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
