package frc.robot.subsystems.feeder;

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
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    limitConfigs.StatorCurrentLimit = 50;
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
  public void updateInputs(FeederIOInputs inputs) {
    inputs.feedermotor1voltage.mut_replace(Motor.getMotorVoltage().getValue());
    inputs.feedermotor1velocity.mut_replace(Motor.getVelocity().getValue());
    inputs.feedermotor1supplyCurrent.mut_replace(Motor.getSupplyCurrent().getValue());
    inputs.feedermotor1statorCurrent.mut_replace(Motor.getStatorCurrent().getValue());
    inputs.feedermotor1torqueCurrent.mut_replace(Motor.getTorqueCurrent().getValue());
    inputs.feedermotor1Temp.mut_replace(Motor.getDeviceTemp().getValue());
    inputs.feedermotor2voltage.mut_replace(Motor2.getMotorVoltage().getValue());
    inputs.feedermotor2velocity.mut_replace(Motor2.getVelocity().getValue());
    inputs.feedermotor2supplyCurrent.mut_replace(Motor2.getSupplyCurrent().getValue());
    inputs.feedermotor2statorCurrent.mut_replace(Motor2.getStatorCurrent().getValue());
    inputs.feedermotor2torqueCurrent.mut_replace(Motor2.getTorqueCurrent().getValue());
    inputs.feedermotor2Temp.mut_replace(Motor2.getDeviceTemp().getValue());
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
