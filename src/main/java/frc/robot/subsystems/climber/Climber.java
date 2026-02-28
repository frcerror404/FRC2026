package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private static final double CLIMBER_UP_SPEED = 0.2;
  private static final double CLIMBER_DOWN_SPEED = -0.2;

  private final TalonFX climberMotor;

  private final VoltageOut rollerRequest = new VoltageOut(0);
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

  // CAN IDs
  public Climber(int climberID) {
    climberMotor = new TalonFX(climberID);

    configureClimber();
  }

  private void configureClimber() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(60)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(35)
                    .withSupplyCurrentLimitEnable(true));

    climberMotor.getConfigurator().apply(config);
  }

  // Climber Up
  public void climberUp() {
    climberMotor.setControl(new DutyCycleOut(CLIMBER_UP_SPEED));
  }

  // Climber Down
  public void climberDown() {
    climberMotor.setControl(new DutyCycleOut(CLIMBER_DOWN_SPEED));
  }

  public void stop() {
    climberMotor.setControl(new DutyCycleOut(0.0));
  }
}
