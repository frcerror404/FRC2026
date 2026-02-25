package frc.robot.subsystems.shooter;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private static final double SHOOTER_SPEED = 0.7;
  private static final double FEEDER_SPEED = 0.6;

  // Flywheel motors
  private final TalonFX leftMotor;
  private final TalonFX middleMotor;
  private final TalonFX rightMotor;
  private final List<TalonFX> shooterMotors;

  // Feeder motor 
  private final TalonFX feederMotor;

  private final VoltageOut voltageRequest = new VoltageOut(0);

  public Shooter(int leftID, int middleID, int rightID, int feederID) {

    leftMotor = new TalonFX(leftID);
    middleMotor = new TalonFX(middleID);
    rightMotor = new TalonFX(rightID);
    feederMotor = new TalonFX(feederID);

    shooterMotors = List.of(leftMotor, middleMotor, rightMotor);

    configureShooterMotor(leftMotor, InvertedValue.CounterClockwise_Positive);
    configureShooterMotor(middleMotor, InvertedValue.Clockwise_Positive);
    configureShooterMotor(rightMotor, InvertedValue.Clockwise_Positive);

    configureFeederMotor(feederMotor);
  }

  private void configureShooterMotor(TalonFX motor, InvertedValue inversion) {

    TalonFXConfiguration config = new TalonFXConfiguration()
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

  private void configureFeederMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true));

    motor.getConfigurator().apply(config);
  }


//Flywheel
  public void runShooter() {
    for (TalonFX motor : shooterMotors) {
      motor.setControl(voltageRequest.withOutput(SHOOTER_SPEED * 12.0));
    }
  }

  public void stopShooter() {
    for (TalonFX motor : shooterMotors) {
      motor.setControl(voltageRequest.withOutput(0.0));
    }
  }

  //Feeder 

  public void runFeeder() {
    feederMotor.setControl(voltageRequest.withOutput(FEEDER_SPEED * 12.0));
  }

  public void stopFeeder() {
    feederMotor.setControl(voltageRequest.withOutput(0.0));
  }


  public void stopAll() {
    stopShooter();
    stopFeeder();
  }
}