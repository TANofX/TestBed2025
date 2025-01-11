// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.pid;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

/** Add your docs here. */
public class TunableSparkPIDController implements TunablePID {
    private final SparkBase device;
    private final ClosedLoopSlot slot;
    private final ClosedLoopConfig config;
    private final ClosedLoopConfigAccessor accessor;
    private final SparkBaseConfig deviceConfig;
    
    public TunableSparkPIDController(SparkFlex device, ClosedLoopSlot pidSlot) {
        this.device  = device;
        slot = pidSlot;
        deviceConfig = new SparkFlexConfig();
        config = deviceConfig.closedLoop;
        accessor = device.configAccessor.closedLoop;
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
        device.configure(deviceConfig, null, null);
    }

    @Override
    public void setFF(double kFF) {
        config.velocityFF(kFF, slot);
        device.configure(deviceConfig, null, null);
    }

    @Override
    public void setI(double kI) {
        config.i(kI, slot);
        device.configure(deviceConfig, null, null);
    }

    @Override
    public void setIZone(double Izone) {
        config.iZone(Izone, slot);
        device.configure(deviceConfig, null, null);
    }

    @Override
    public void setOutputRange(double minOutput, double maxOutput) {
        config.outputRange(minOutput, maxOutput, slot);
        device.configure(deviceConfig, null, null);
    }

    @Override
    public void setP(double kP) {
        config.p(kP, slot);
        device.configure(deviceConfig, null, null);
    }
}

