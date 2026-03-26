package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public MutVoltage shootermotor1voltage;
    public MutAngularVelocity shootermotor1velocity;
    public MutCurrent shootermotor1supplyCurrent;
    public MutCurrent shootermotor1statorCurrent;
    public MutCurrent shootermotor1torqueCurrent;
    public MutTemperature shootermotor1Temp;
    public MutVoltage shootermotor2voltage;
    public MutAngularVelocity shootermotor2velocity;
    public MutCurrent shootermotor2supplyCurrent;
    public MutCurrent shootermotor2statorCurrent;
    public MutCurrent shootermotor2torqueCurrent;
    public MutTemperature shootermotor2Temp;
    public MutVoltage shootermotor3voltage;
    public MutAngularVelocity shootermotor3velocity;
    public MutCurrent shootermotor3supplyCurrent;
    public MutCurrent shootermotor3statorCurrent;
    public MutCurrent shootermotor3torqueCurrent;
    public MutTemperature shootermotor3Temp;
    public MutVoltage shootermotor4voltage;
    public MutAngularVelocity shootermotor4velocity;
    public MutCurrent shootermotor4supplyCurrent;
    public MutCurrent shootermotor4statorCurrent;
    public MutCurrent shootermotor4torqueCurrent;
    public MutTemperature shootermotor4Temp;

    public double shotSpeed;
  }

  public void shootFuel(double shotSpeed);

  public void updateInputs(ShooterIOInputs input);

  public void shooterPID(Gains gains);

  public void stop();
}
