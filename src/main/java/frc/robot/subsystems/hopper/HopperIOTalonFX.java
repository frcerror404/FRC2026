package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.CanDef;
import frc.robot.util.PhoenixUtil;

public class HopperIOTalonFX implements HopperIO {
  public TalonFX Motor;
  public double hopperSpeed;

  public HopperIOTalonFX(CanDef canbus) {
    Motor = new TalonFX(canbus.id(), canbus.bus());

    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.CurrentLimits.StatorCurrentLimit = 80.0;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.Voltage.PeakForwardVoltage = 12.0;
    cfg.Voltage.PeakReverseVoltage = -12.0;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));
  }

  @Override
  public void updateInputs(HopperIO.HopperIOInputs inputs) {
    inputs.angularVelocity.mut_replace(Motor.getVelocity().getValue());
    inputs.voltage.mut_replace(Motor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(Motor.getSupplyCurrent().getValue());
    inputs.torqueCurrent.mut_replace(Motor.getTorqueCurrent().getValue());
  }

  @Override
  public void runHopper(double hopperSpeed) {
    Motor.setControl(new VelocityVoltage(hopperSpeed));
  }

  @Override
  public void stop() {
    Motor.setControl(new StaticBrake());
  }
}
