package frc.robot.subsystems.shooterReverse;

import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterReverseIO {

  @AutoLog
  public static class ShooterReverseIOInputs {
    public MutVoltage voltage;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
    public double shotSpeed;
  }

  public void shootFuel(double shotSpeed);

  public void updateInputs(ShooterReverseIOInputs input);

  public void stop();
}
