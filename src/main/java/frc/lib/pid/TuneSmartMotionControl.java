// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.pid;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public class TuneSmartMotionControl extends TuneSparkPIDController {
    private double minVel, maxVel, maxAcc, allowedErr;

    public TuneSmartMotionControl(String motorName, SparkBase sparkMotor, Subsystem motorOwner) {
        super(motorName, sparkMotor, motorOwner);
    }

    @Override
    public void execute() {
        super.execute();

        double maxV = SmartDashboard.getNumber(name + " Max Velocity", 0);
        double minV = SmartDashboard.getNumber(name + " Min Velocity", 0);
        double maxA = SmartDashboard.getNumber(name + " Max Acceleration", 0);
        double allE = SmartDashboard.getNumber(name + " Allowed Closed Loop Error", 0);

        if ((maxV != maxVel)) {
            pidController.setSmartMotionMaxVelocity(maxV, 0);
            maxVel = maxV;
        }
        if ((minV != minVel)) {
            pidController.setSmartMotionMinOutputVelocity(minV, 0);
            minVel = minV;
        }
        if ((maxA != maxAcc)) {
            pidController.setSmartMotionMaxAccel(maxA, 0);
            maxAcc = maxA;
        }
        if ((allE != allowedErr)) {
            pidController.setSmartMotionAllowedClosedLoopError(allE, 0);
            allowedErr = allE;
        }

        double setPoint, processVariable;
        boolean mode = SmartDashboard.getBoolean(name + " Mode", false);
        if (mode) {
            setPoint = SmartDashboard.getNumber(name + " Set Velocity", 0);
            pidController.setReference(setPoint, SparkMax.ControlType.kVelocity);
            processVariable = encoder.getVelocity();
        } else {
            setPoint = SmartDashboard.getNumber(name + " Set Position", 0);
            /**
             * As with other PID modes, Smart Motion is set by calling the
             * setReference method on an existing pid object and setting
             * the control type to kSmartMotion
             */
            pidController.setReference(setPoint, SparkMax.ControlType.kSmartMotion);
            processVariable = encoder.getPosition();
        }

        SmartDashboard.putNumber(name + " SetPoint", setPoint);
        SmartDashboard.putNumber(name + " Process Variable", processVariable);
        SmartDashboard.putNumber(name + " Output", tuningController.getAppliedOutput());
        SmartDashboard.putNumber(name + " Current Position", encoder.getPosition());

    }

    @Override
    public void initialize() {
        super.initialize();

        // Smart Motion Coefficients
        maxVel = 2000; // rpm
        maxAcc = 1500;
        allowedErr = 0;
        /**
         * Smart Motion coefficients are set on a SparkPIDController object
         * 
         * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
         * the pid controller in Smart Motion mode
         * - setSmartMotionMinOutputVelocity() will put a lower bound in
         * RPM of the pid controller in Smart Motion mode
         * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
         * of the pid controller in Smart Motion mode
         * - setSmartMotionAllowedClosedLoopError() will set the max allowed
         * error for the pid controller in Smart Motion mode
         */
        int smartMotionSlot = 0;
        pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        //encoder.setPosition(0.0);

        // display Smart Motion coefficients
        SmartDashboard.putNumber(name + " Max Velocity", maxVel);
        SmartDashboard.putNumber(name + " Min Velocity", minVel);
        SmartDashboard.putNumber(name + " Max Acceleration", maxAcc);
        SmartDashboard.putNumber(name + " Allowed Closed Loop Error", allowedErr);
        SmartDashboard.putNumber(name + " Set Position", 0);
        SmartDashboard.putNumber(name + " Set Velocity", 0);

        // button to toggle between velocity and smart motion modes
        SmartDashboard.putBoolean(name + " Mode", true);

    }
}
