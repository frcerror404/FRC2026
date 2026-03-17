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
    public double shotSpeed;
    public boolean isReverse;
  }

  public void shootFuel(double shotSpeed, boolean isReverse);

  public void updateInputs(ShooterIOInputs input);

  public void stop();
}
