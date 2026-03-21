package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Volts;

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

public class FeederIOTalonFX implements FeederIO {
  public VoltageOut Request;
  public TalonFX Motor;
  public TalonFX Motor2;
  public double feederSpeed;

  private Voltage m_setPoint = Voltage.ofBaseUnits(0, Volts);

  public FeederIOTalonFX(CanDef canbus, CanDef canbus2) {
    Motor = new TalonFX(canbus.id());
    Motor2 = new TalonFX(canbus2.id());
    Request = new VoltageOut(0.0);

    Motor2.setControl(new Follower(Motor.getDeviceID(), MotorAlignmentValue.Aligned));

    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // cfg.CurrentLimits.StatorCurrentLimit = 80.0;
    // cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    // cfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    // cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    // cfg.Voltage.PeakForwardVoltage = -12.0;
    // cfg.Voltage.PeakReverseVoltage = 12.0;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.angularVelocity.mut_replace(Motor.getVelocity().getValue());
    inputs.voltageSetPoint.mut_replace(m_setPoint);
    inputs.voltage.mut_replace(Motor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(Motor.getSupplyCurrent().getValue());
  }

  @Override
  public void runFeeder(double feederSpeed) {
    Motor.setControl(new VoltageOut(feederSpeed).withEnableFOC(true));
  }

  @Override
  public void stop() {
    Motor.setControl(new StaticBrake());
  }
}
