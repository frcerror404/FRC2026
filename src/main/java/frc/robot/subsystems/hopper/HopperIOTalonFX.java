package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.CanDef;
import frc.robot.util.PhoenixUtil;

public class HopperIOTalonFX implements HopperIO {
  public VoltageOut Request;
  public TalonFX Motor;
  public double hopperSpeed;

  private Voltage m_setPoint = Voltage.ofBaseUnits(0, Volts);

  public HopperIOTalonFX(CanDef canbus) {
    Motor = new TalonFX(canbus.id());
    Request = new VoltageOut(0.0);

    configureTalons();
  }

  private void configureTalons() {

    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    limitConfigs.StatorCurrentLimit = 40;
    limitConfigs.StatorCurrentLimitEnable = true;
    limitConfigs.SupplyCurrentLimit = 25;
    limitConfigs.StatorCurrentLimitEnable = true;

    motorOutputConfigs.withInverted(InvertedValue.Clockwise_Positive);
    motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);

    final TalonFXConfiguration commonConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(motorOutputConfigs)
            .withCurrentLimits(limitConfigs);
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(commonConfigs));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.angularVelocity.mut_replace(Motor.getVelocity().getValue());
    inputs.voltageSetPoint.mut_replace(m_setPoint);
    inputs.voltage.mut_replace(Motor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(Motor.getSupplyCurrent().getValue());
  }

  @Override
  public void runHopper(double hopperSpeed) {
    Motor.setControl(new VoltageOut(hopperSpeed));
  }

  @Override
  public void stop() {
    Motor.setControl(new StaticBrake());
  }
}
