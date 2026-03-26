package frc.robot.subsystems.feeder;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

  @AutoLog
  public static class FeederIOInputs {
    public MutVoltage feedermotor1voltage;
    public MutAngularVelocity feedermotor1velocity;
    public MutCurrent feedermotor1supplyCurrent;
    public MutCurrent feedermotor1statorCurrent;
    public MutCurrent feedermotor1torqueCurrent;
    public MutTemperature feedermotor1Temp;

    public MutVoltage feedermotor2voltage;
    public MutAngularVelocity feedermotor2velocity;
    public MutCurrent feedermotor2supplyCurrent;
    public MutCurrent feedermotor2statorCurrent;
    public MutCurrent feedermotor2torqueCurrent;
    public MutTemperature feedermotor2Temp;

    public double feederSpeed;
  }

  public void runFeeder(double feederSpeed);

  public void updateInputs(FeederIOInputs input);

  public void stop();
}
