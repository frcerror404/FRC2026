package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public MutVoltage intakemotor1voltage;
    public MutAngularVelocity intakemotor1velocity;
    public MutCurrent intakemotor1supplyCurrent;
    public MutCurrent intakemotor1statorCurrent;
    public MutCurrent intakemotor1torqueCurrent;
    public MutTemperature intakemotor1Temp;

    public MutVoltage intakemotor2voltage;
    public MutAngularVelocity intakemotor2velocity;
    public MutCurrent intakemotor2supplyCurrent;
    public MutCurrent intakemotor2statorCurrent;
    public MutCurrent intakemotor2torqueCurrent;
    public MutTemperature intakemotor2Temp;

    public double intakeSpeed;
  }

  public void setTarget(Voltage target);

  public void runIntake(double intakeSpeed);

  public void updateInputs(IntakeIOInputs input);

  public void stop();
}
