package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  @AutoLog
  public static class HopperIOInputs {
    public MutAngularVelocity angularVelocity;
    public MutVoltage voltage;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
    public double hopperSpeed;
  }

  public void runHopper(double hopperSpeed);

  public void updateInputs(HopperIOInputs input);

  public void stop();
}
