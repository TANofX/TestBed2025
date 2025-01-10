package frc.lib.subsystem.selfcheck;


import com.ctre.phoenix6.hardware.Pigeon2;
import frc.lib.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingPigeon2 implements SelfChecking {
  private final String label;
  private final Pigeon2 pigeon;

  public SelfCheckingPigeon2(String label, Pigeon2 pigeon) {
    this.label = label;
    this.pigeon = pigeon;
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();


    if (pigeon.getFault_Hardware().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware fault detected", label)));
    }
    if (pigeon.getFault_BootDuringEnable().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (pigeon.getFault_BootIntoMotion().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while in motion", label)));
    }
    if (pigeon.getFault_BootupGyroscope().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Gyro fault detected", label)));
    }
    if (pigeon.getFault_BootupAccelerometer().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Accelerometer fault detected", label)));
    }
    
    return faults;
  }
}
