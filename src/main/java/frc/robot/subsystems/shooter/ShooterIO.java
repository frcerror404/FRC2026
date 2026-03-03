package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public MutVoltage voltage;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void shootFuel(double shotSpeed);

  public void updateInputs(ShooterIOInputs input);

  public void stop();
}
