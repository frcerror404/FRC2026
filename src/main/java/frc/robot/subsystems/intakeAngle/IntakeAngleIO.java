package frc.robot.subsystems.intakeAngle;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeAngleIO {

  @AutoLog
  public static class IntakeAngleIOInputs {
    public MutAngle intakeAngle;
    public MutAngularVelocity intakeAngularVelocity;
    public MutAngle intakeAngleSetPoint;
    public MutVoltage voltage;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void setTarget(Angle target);

  public void applyCoastMode();

  public void updateInputs(IntakeAngleIOInputs inputs);

  public void setGains(Gains gains);
}
