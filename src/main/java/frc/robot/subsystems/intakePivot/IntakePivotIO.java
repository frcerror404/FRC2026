package frc.robot.subsystems.intakePivot;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {

  @AutoLog
  public static class IntakePivotIOInputs {
    public MutAngularVelocity angularVelocity;
    public MutVoltage voltage;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
    public MutAngle intakeAngle;
  }

  public void updateInputs(IntakePivotIOInputs input);

  public void stop();

  public void setBrakeMode(boolean Enabled);

  public Command pivotToStow();

  public Command pivotToIntake();
}
