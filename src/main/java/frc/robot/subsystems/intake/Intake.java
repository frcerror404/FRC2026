package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  // Change once robot is finished
  private static final double PIVOT_INTAKE_ANGLE = -10;
  private static final double PIVOT_STOW_ANGLE = 90;

  private static final double PIVOT_GEAR_RATIO = 50.0;

  private static final double INTAKE_SPEED = 0.8;
  private static final double OUTTAKE_SPEED = -0.8;

  private final TalonFX rollerMotor;
  private final TalonFX pivotMotor;

  private final VoltageOut rollerRequest = new VoltageOut(0);
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

  // CAN IDs
  public Intake(int rollerID, int pivotID) {
    rollerMotor = new TalonFX(rollerID);
    pivotMotor = new TalonFX(pivotID);

    configureRoller();
    configurePivot();

    pivotMotor.setPosition(Degrees.of(PIVOT_STOW_ANGLE));
  }

  private void configureRoller() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(20)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(35)
                    .withSupplyCurrentLimitEnable(true));

    rollerMotor.getConfigurator().apply(config);
  }

  private void configurePivot() {

    AngularVelocity maxSpeed = RotationsPerSecond.of(100).div(PIVOT_GEAR_RATIO);

    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
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

  // Roller Control
  public void intakeIn() {
    rollerMotor.setControl(new DutyCycleOut(INTAKE_SPEED));
  }

  public void intakeOut() {
    rollerMotor.setControl(new DutyCycleOut(OUTTAKE_SPEED));
  }

  public void stop() {
    rollerMotor.setControl(new DutyCycleOut(0.0));
  }

  // Pivot Control

  public void pivotToIntake() {
    pivotMotor.setControl(pivotRequest.withPosition(Degrees.of(PIVOT_INTAKE_ANGLE)));
  }

  public void pivotToStow() {
    pivotMotor.setControl(pivotRequest.withPosition(Degrees.of(PIVOT_STOW_ANGLE)));
  }
}
