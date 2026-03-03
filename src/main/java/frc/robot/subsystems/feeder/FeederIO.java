package frc.robot.subsystems.feeder;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

  @AutoLog
  public static class FeederIOInputs {
    public MutAngularVelocity angularVelocity;
    public MutVoltage voltage;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
    public double feederSpeed;
  }

  public void runFeeder(double feederSpeed);

  public void updateInputs(FeederIOInputs input);

  public void stop();
}
