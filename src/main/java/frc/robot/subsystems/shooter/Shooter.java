package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

public class Shooter extends SubsystemBase {

  private static final double SHOOTER_SPEED = 0.7;

  private final TalonFX leftMotor;
  private final TalonFX middleMotor;
  private final TalonFX rightMotor;

  private final List<TalonFX> motors;

  private final VoltageOut voltageRequest = new VoltageOut(0);

  public Shooter(int leftID, int middleID, int rightID) {

    leftMotor = new TalonFX(leftID);
    middleMotor = new TalonFX(middleID);
    rightMotor = new TalonFX(rightID);

    motors = List.of(leftMotor, middleMotor, rightMotor);

    configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive);
    configureMotor(middleMotor, InvertedValue.Clockwise_Positive);
    configureMotor(rightMotor, InvertedValue.Clockwise_Positive);
  }

  private void configureMotor(TalonFX motor, InvertedValue inversion) {

    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(inversion)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(80)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true));

    motor.getConfigurator().apply(config);
  }

  public void shooterOut() {
    setPercentOutput(SHOOTER_SPEED);
  }

  public void setPercentOutput(double percent) {
    for (TalonFX motor : motors) {
      motor.setControl(voltageRequest.withOutput(percent * 12.0));
    }
  }

  public void stop() {
    setPercentOutput(0.0);
  }
}
