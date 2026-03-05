package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {

  // Change once robot is finished
  private static final double PIVOT_INTAKE_ANGLE = -15;
  private static final double PIVOT_STOW_ANGLE = 115;
  private static final double PIVOT_AGITATE_ANGLE = 35;

  private static final double PIVOT_GEAR_RATIO = 50.0;

  private final TalonFX pivotMotor;

  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

  // CAN IDs
  public IntakePivot(int pivotID) {
    pivotMotor = new TalonFX(pivotID);

    configurePivot();

    pivotMotor.setPosition(Degrees.of(PIVOT_STOW_ANGLE));
  }

  private void configurePivot() {

    AngularVelocity maxSpeed = RotationsPerSecond.of(100).div(PIVOT_GEAR_RATIO);

    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(20)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(10)
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(PIVOT_GEAR_RATIO))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(maxSpeed)
                    .withMotionMagicAcceleration(maxSpeed.per(Second)))
            .withSlot0(
                new Slot0Configs()
                    .withKP(150)
                    .withKD(0)
                    .withKV(12.0 / maxSpeed.in(RotationsPerSecond)));

    pivotMotor.getConfigurator().apply(config);
  }
  // Pivot Control

  public void pivotToIntake() {
    pivotMotor.setControl(pivotRequest.withPosition(Degrees.of(PIVOT_INTAKE_ANGLE)));
  }

  public void agitateIntake() {
    pivotMotor.setControl(pivotRequest.withPosition(Degrees.of(PIVOT_AGITATE_ANGLE)));
  }

  public void pivotToStow() {
    pivotMotor.setControl(pivotRequest.withPosition(Degrees.of(PIVOT_STOW_ANGLE)));
  }
}
