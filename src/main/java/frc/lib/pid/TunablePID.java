// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.pid;

/** Add your docs here. */
public interface TunablePID {
    double getP();
    double getI();
    double getD();
    double getFF();
    double getIZone();
    double getOutputMin();
    double getOutputMax();

    void setP(double kP);
    void setI(double kI);
    void setD(double kD);
    void setFF(double kFF);
    void setIZone(double Izone);
    void setOutputRange(double minOutput, double maxOutput);
}
