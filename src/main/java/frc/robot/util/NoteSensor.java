// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class NoteSensor extends AnalogInput {
     private double triggerVoltage = 0.9;

    public boolean isTriggered () {
        SmartDashboard.putNumber("Sensor " + super.getChannel(), super.getVoltage());
        return getVoltage() >= triggerVoltage;
    }
    public NoteSensor(int channel) {
        super(channel);
    }
    public NoteSensor(int channel, double voltage) {
        this (channel);
        triggerVoltage = voltage;
    }
}
