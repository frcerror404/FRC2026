package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  @AutoLog
  public static class HopperIOInputs {
    public MutAngularVelocity hoppervelocity;
    public MutVoltage hoppervoltage;
    public MutVoltage hoppervoltageSetPoint;
    public MutCurrent hoppersupplyCurrent;
    public MutCurrent hoppertorqueCurrent;
    public MutCurrent hopperstatorCurrent;
    public MutTemperature hoppertemperature;

    public double hopperSpeed;
  }

  public void runHopper(double hopperSpeed);

  public void updateInputs(HopperIOInputs input);

  public void stop();
}
