// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDs.AnimationTypes;

public class Notifications extends Command {
  private enum LED_State {
    FC_ACTIVE,
    FC_SPEAKER_READY,
    FC_SHOOTER_READY,
    AMP_READY,
    HAS_NOTE,
    DEFAULT
  }

  private LED_State currentState;
  private LED_State priorState;

  /** Creates a new Notifications. */
  public Notifications() {
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(RobotContainer.LEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    priorState = LED_State.HAS_NOTE;
    currentState = LED_State.DEFAULT;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


   SmartDashboard.putNumber("Total Current", RobotContainer.powerDistribution.getTotalCurrent());
   currentState = LED_State.DEFAULT;

   if (priorState != currentState) {
    switch (currentState) {
      default:
        RobotContainer.LEDs.changeAnimation(AnimationTypes.OneColorGreen);
        break;
    }
   }

   priorState = currentState;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.LEDs.changeAnimation(AnimationTypes.Rainbow);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
