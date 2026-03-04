// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakePivot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.CanDef;
import edu.wpi.first.units.measure.*;

public class IntakePivotIOTalonFX implements IntakePivotIO {
  public TalonFX pivotMotor;
  private MotionMagicVoltage pivotRequest;

  private static final double PIVOT_INTAKE_ANGLE = -10;
  private static final double PIVOT_STOW_ANGLE = 90;
  private static final double PIVOT_GEAR_RATIO = 50.0;

  public IntakePivotIOTalonFX(CanDef canbus) {
    pivotMotor = new TalonFX(canbus.id(), canbus.bus());

    configureTalons();
  }

  private void configureTalons() {
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

    pivotMotor.setPosition(Degrees.of(PIVOT_STOW_ANGLE));
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.angularVelocity.mut_replace(pivotMotor.getVelocity().getValue());
    inputs.voltage.mut_replace(pivotMotor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(pivotMotor.getSupplyCurrent().getValue());
    inputs.torqueCurrent.mut_replace(pivotMotor.getTorqueCurrent().getValue());
    inputs.intakeAngle.mut_replace(pivotMotor.getPosition().getValue());
  }

  @Override
  public Command stop() {
    return new InstantCommand(
        () -> {
          pivotMotor.setControl(new StaticBrake());
        });
  }

  public Command pivotToIntake() {
    return new InstantCommand(
        () -> {
          pivotMotor.setControl(pivotRequest.withPosition(Degrees.of(PIVOT_INTAKE_ANGLE)));
        });
  }

  public Command pivotToStow() {
    return new InstantCommand(
        () -> {
          pivotMotor.setControl(pivotRequest.withPosition(Degrees.of(PIVOT_STOW_ANGLE)));
        });
  }

  public void setBrakeMode(boolean Enabled) {
    MotorOutputConfigs mConfigs =
        new MotorOutputConfigs()
            .withNeutralMode(Enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive);
    pivotMotor.getConfigurator().apply(mConfigs);
  }
}
