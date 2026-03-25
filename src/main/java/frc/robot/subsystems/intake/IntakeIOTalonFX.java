package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.CanDef;
import frc.robot.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {
  public VoltageOut Request;
  public TalonFX Motor;
  public TalonFX Motor2;
  public double intakeSpeed;
  public CurrentLimitsConfigs limitConfigs;
  public MotorOutputConfigs motorOutputConfigs;

  private Voltage m_setPoint = Voltage.ofBaseUnits(0, Volts);

  public IntakeIOTalonFX(CanDef canbus, CanDef canbus2) {
    Motor = new TalonFX(canbus.id());
    Motor2 = new TalonFX(canbus2.id());
    Request = new VoltageOut(0.0);

    Motor2.setControl(new Follower(Motor.getDeviceID(), MotorAlignmentValue.Opposed));

    configureTalons();
  }

  private void configureTalons() {
    limitConfigs = new CurrentLimitsConfigs();
    motorOutputConfigs = new MotorOutputConfigs();

    limitConfigs.StatorCurrentLimit = 60;
    limitConfigs.StatorCurrentLimitEnable = true;
    limitConfigs.SupplyCurrentLimit = 30;
    limitConfigs.StatorCurrentLimitEnable = true;

    motorOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive);
    motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);

    final TalonFXConfiguration commonConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(motorOutputConfigs)
            .withCurrentLimits(limitConfigs);

    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(commonConfigs));
  }

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    inputs.angularVelocity.mut_replace(Motor.getVelocity().getValue());
    inputs.voltageSetPoint.mut_replace(m_setPoint);
    inputs.voltage.mut_replace(Motor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(Motor.getSupplyCurrent().getValue());
  }

  @Override
  public void setTarget(Voltage target) {
    Request = Request.withOutput(target);
    Motor.setControl(Request);
    m_setPoint = target;
  }

  @Override
  public void runIntake(double intakeSpeed) {
    Motor.setControl(new VoltageOut(intakeSpeed));
  }

  @Override
  public void stop() {
    Motor.setControl(new StaticBrake());
  }
}
