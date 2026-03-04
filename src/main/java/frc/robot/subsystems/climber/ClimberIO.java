package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public MutAngularVelocity angularVelocity;
    public MutVoltage voltage;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
    public double hopperSpeed;
  }

  public void runClimber(double climberSpeed);

  public void updateInputs(ClimberIOInputs input);

  public void stop();
}
