package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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
  public TalonFX Motor2;
  public Slot0Configs Slot0Configs;
  public MotorOutputConfigs motorOutputConfigs;
  public CurrentLimitsConfigs limitConfigs;
  public double shotSpeed;
  public boolean isReverse;

  public ShooterIOTalonFX(CanDef canbus) {
    Motor = new TalonFX(canbus.id());
    Motor2 = new TalonFX(canbus.id());

    shooterPID();
    configureTalons();
  }

  private void configureTalons() {
    limitConfigs = new CurrentLimitsConfigs();
    motorOutputConfigs = new MotorOutputConfigs();
    var reverseOutputsConfigs = new MotorOutputConfigs();

    reverseOutputsConfigs.withInverted(InvertedValue.Clockwise_Positive);

    limitConfigs.StatorCurrentLimit = 20;
    limitConfigs.StatorCurrentLimitEnable = false;

    motorOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive);
    motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);

    final TalonFXConfiguration commonConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(motorOutputConfigs)
            .withCurrentLimits(limitConfigs);

    final TalonFXConfiguration reverseConfigs =
        commonConfigs.clone().withMotorOutput(reverseOutputsConfigs);

    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(commonConfigs));
    PhoenixUtil.tryUntilOk(5, () -> Motor2.getConfigurator().apply(reverseConfigs));
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
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.voltage.mut_replace(Motor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(Motor.getSupplyCurrent().getValue());
  }

  @Override
  public void shootFuel(double shotSpeed, boolean isReverse) {
    if (isReverse) {
      Motor.setControl(new VoltageOut(shotSpeed));
    } else {
      Motor2.setControl(new VoltageOut(shotSpeed));
    }
  }

  @Override
  public void stop() {
    Motor.setControl(new StaticBrake());
  }
}
