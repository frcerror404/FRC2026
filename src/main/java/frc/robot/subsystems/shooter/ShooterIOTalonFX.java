package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.CanDef;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
  public VoltageOut Request;
  public TalonFX Motor1;
  public TalonFX Motor2;
  public TalonFX Motor3;
  public Slot0Configs Slot0Configs;
  public MotorOutputConfigs motorOutputConfigs;
  public CurrentLimitsConfigs limitConfigs;
  public double shotSpeed;
  public boolean isReverse;

  public ShooterIOTalonFX(CanDef canbus1, CanDef canbus2, CanDef canbus3) {
    Motor1 = new TalonFX(canbus1.id());
    Motor2 = new TalonFX(canbus2.id());
    Motor3 = new TalonFX(canbus3.id());

    Motor2.setControl(new Follower(Motor1.getDeviceID(), MotorAlignmentValue.Aligned));
    Motor3.setControl(new Follower(Motor1.getDeviceID(), MotorAlignmentValue.Opposed));

    configureTalons();
  }

  private void configureTalons() {
    limitConfigs = new CurrentLimitsConfigs();
    motorOutputConfigs = new MotorOutputConfigs();

    limitConfigs.StatorCurrentLimit = 20;
    limitConfigs.StatorCurrentLimitEnable = false;

    motorOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive);
    motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);

    final TalonFXConfiguration commonConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(motorOutputConfigs)
            .withCurrentLimits(limitConfigs);

    PhoenixUtil.tryUntilOk(5, () -> Motor1.getConfigurator().apply(commonConfigs));
  }

  @Override
  public void shooterPID(Gains gains) {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kV = gains.kV;

    PhoenixUtil.tryUntilOk(5, () -> Motor1.getConfigurator().apply(slot0Configs));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.voltage.mut_replace(Motor1.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(Motor1.getSupplyCurrent().getValue());
  }

  @Override
  public void shootFuel(double shotSpeed) {
      Motor1.setControl(new VoltageOut(shotSpeed).withEnableFOC(true));
  }

  @Override
  public void stop() {
    Motor1.setControl(new StaticBrake());
  }
}
