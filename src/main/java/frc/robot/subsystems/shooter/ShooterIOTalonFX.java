package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.CanDef;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
  public VoltageOut Request;
  public TalonFX Motor;
  public double shotSpeed;

  public ShooterIOTalonFX(CanDef canbus) {
    Motor = new TalonFX(canbus.id(), canbus.bus());

    shooterPID();
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // cfg.CurrentLimits.StatorCurrentLimit = 80.0;
    // cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    // cfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    // cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    // cfg.Voltage.PeakForwardVoltage = 12.0;
    // cfg.Voltage.PeakReverseVoltage = 12.0;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.voltage.mut_replace(Motor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(Motor.getSupplyCurrent().getValue());
  }

  private void shooterPID() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.1;
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.11;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    Motor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void shootFuel(double shotSpeed) {
    Motor.setControl(new VoltageOut(shotSpeed));
  }

  @Override
  public void stop() {
    Motor.setControl(new StaticBrake());
  }
}
