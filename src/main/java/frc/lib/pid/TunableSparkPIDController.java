// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.pid;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;

/** Add your docs here. */
public class TunableSparkPIDController implements TunablePID {
    private final SparkClosedLoopController pidController;
    private final ClosedLoopSlot slot;
    private final ClosedLoopConfig config;
    private final ClosedLoopConfigAccessor accessor;
    
    public TunableSparkPIDController(SparkClosedLoopController pid, ClosedLoopSlot pidSlot) {
        pidController = pid;
        slot = pidSlot;
        config = new ClosedLoopConfig();
        // todo
    }

    @Override
    public double getD() {
        return accessor.getD(slot);
    }

    @Override
    public double getFF() {
        return accessor.getFF(slot);
    }

    @Override
    public double getI() {
        return accessor.getI(slot);
    }

    @Override
    public double getIZone() {
        return accessor.getIZone(slot);
    }

    @Override
    public double getOutputMax() {
        return accessor.getMaxOutput(slot);
    }

    @Override
    public double getOutputMin() {
        return accessor.getMinOutput(slot);
    }

    @Override
    public double getP() {
        return accessor.getP(slot);
    }

    @Override
    public void setD(double kD) {
        config.d(kD, slot);
    }

    @Override
    public void setFF(double kFF) {
        config.velocityFF(kFF, slot);
    }

    @Override
    public void setI(double kI) {
        config.i(kI, slot);
    }

    @Override
    public void setIZone(double Izone) {
        config.iZone(Izone, slot);
    }

    @Override
    public void setOutputRange(double minOutput, double maxOutput) {
        config.outputRange(minOutput, maxOutput, slot);
    }

    @Override
    public void setP(double kP) {
        config.p(kP, slot);     
    }
}

