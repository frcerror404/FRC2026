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
  public TalonFX Motor4;
  public Slot0Configs Slot0Configs;
  public MotorOutputConfigs motorOutputConfigs;
  public CurrentLimitsConfigs limitConfigs;
  public double shotSpeed;
  public boolean isReverse;

  public ShooterIOTalonFX(CanDef canbus1, CanDef canbus2, CanDef canbus3, CanDef canbus4) {
    Motor1 = new TalonFX(canbus1.id());
    Motor2 = new TalonFX(canbus2.id());
    Motor3 = new TalonFX(canbus3.id());
    Motor4 = new TalonFX(canbus4.id());

    Motor2.setControl(new Follower(Motor1.getDeviceID(), MotorAlignmentValue.Aligned));
    Motor3.setControl(new Follower(Motor1.getDeviceID(), MotorAlignmentValue.Opposed));
    Motor4.setControl(new Follower(Motor1.getDeviceID(), MotorAlignmentValue.Opposed));

    configureTalons();
  }

  private void configureTalons() {
    limitConfigs = new CurrentLimitsConfigs();
    motorOutputConfigs = new MotorOutputConfigs();

    limitConfigs.StatorCurrentLimit = 80;
    limitConfigs.StatorCurrentLimitEnable = true;
    limitConfigs.SupplyCurrentLimit = 40;
    limitConfigs.StatorCurrentLimitEnable = true;

    motorOutputConfigs.withInverted(InvertedValue.Clockwise_Positive);
    motorOutputConfigs.withNeutralMode(NeutralModeValue.Coast);

    final TalonFXConfiguration commonConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(motorOutputConfigs)
            .withCurrentLimits(limitConfigs);

    PhoenixUtil.tryUntilOk(5, () -> Motor1.getConfigurator().apply(commonConfigs));
    PhoenixUtil.tryUntilOk(5, () -> Motor2.getConfigurator().apply(commonConfigs));
    PhoenixUtil.tryUntilOk(5, () -> Motor3.getConfigurator().apply(commonConfigs));
    PhoenixUtil.tryUntilOk(5, () -> Motor4.getConfigurator().apply(commonConfigs));
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
    inputs.shootermotor1voltage.mut_replace(Motor1.getMotorVoltage().getValue());
    inputs.shootermotor1velocity.mut_replace(Motor1.getVelocity().getValue());
    inputs.shootermotor1supplyCurrent.mut_replace(Motor1.getSupplyCurrent().getValue());
    inputs.shootermotor1statorCurrent.mut_replace(Motor1.getStatorCurrent().getValue());
    inputs.shootermotor1torqueCurrent.mut_replace(Motor1.getTorqueCurrent().getValue());
    inputs.shootermotor1Temp.mut_replace(Motor1.getDeviceTemp().getValue());
    inputs.shootermotor2voltage.mut_replace(Motor2.getMotorVoltage().getValue());
    inputs.shootermotor2velocity.mut_replace(Motor2.getVelocity().getValue());
    inputs.shootermotor2supplyCurrent.mut_replace(Motor2.getSupplyCurrent().getValue());
    inputs.shootermotor2statorCurrent.mut_replace(Motor2.getStatorCurrent().getValue());
    inputs.shootermotor2torqueCurrent.mut_replace(Motor2.getTorqueCurrent().getValue());
    inputs.shootermotor2Temp.mut_replace(Motor2.getDeviceTemp().getValue());
    inputs.shootermotor3voltage.mut_replace(Motor3.getMotorVoltage().getValue());
    inputs.shootermotor3velocity.mut_replace(Motor3.getVelocity().getValue());
    inputs.shootermotor3supplyCurrent.mut_replace(Motor3.getSupplyCurrent().getValue());
    inputs.shootermotor3statorCurrent.mut_replace(Motor3.getStatorCurrent().getValue());
    inputs.shootermotor3torqueCurrent.mut_replace(Motor3.getTorqueCurrent().getValue());
    inputs.shootermotor3Temp.mut_replace(Motor3.getDeviceTemp().getValue());
    inputs.shootermotor4voltage.mut_replace(Motor4.getMotorVoltage().getValue());
    inputs.shootermotor4velocity.mut_replace(Motor4.getVelocity().getValue());
    inputs.shootermotor4supplyCurrent.mut_replace(Motor4.getSupplyCurrent().getValue());
    inputs.shootermotor4statorCurrent.mut_replace(Motor4.getStatorCurrent().getValue());
    inputs.shootermotor4torqueCurrent.mut_replace(Motor4.getTorqueCurrent().getValue());
    inputs.shootermotor4Temp.mut_replace(Motor4.getDeviceTemp().getValue());
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
