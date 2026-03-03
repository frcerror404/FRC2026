package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public MutAngularVelocity angularVelocity;
    public MutVoltage voltage;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
    public double intakeSpeed;
  }

  public void setTarget(Voltage target);

  public void runIntake(double intakeSpeed);

  public void runIntakeReverse(double intakeSpeed);

  public void updateInputs(IntakeIOInputs input);

  public void stop();
}
