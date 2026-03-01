package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {

  // private static final double SHOOTER_SPEED = -0.7;
  // private static final double FEEDER_SPEED = -1.0;
  private final ShooterIO m_ShooterIO;

  ShooterIOInputsAutoLogged loggedshooter = new ShooterIOInputsAutoLogged();

  // Flywheel motors
  // private final TalonFX leftMotor;
  // private final TalonFX middleMotor;
  // private final TalonFX rightMotor;
  // private final List<TalonFX> shooterMotors;

  // Feeder motor
  // private final TalonFX feederMotor;

  // private final VoltageOut voltageRequest = new VoltageOut(0);

  public Shooter(ShooterIO ShooterIO) {
    m_ShooterIO = ShooterIO;
    // leftMotor = new TalonFX(leftID);
    // middleMotor = new TalonFX(middleID);
    // rightMotor = new TalonFX(rightID);
    // feederMotor = new TalonFX(feederID);

    loggedshooter.angularVelocity = DegreesPerSecond.mutable(0);
    loggedshooter.supplyCurrent = Amps.mutable(0);
    loggedshooter.torqueCurrent = Amps.mutable(0);
    loggedshooter.voltageSetPoint = Volts.mutable(0);
    loggedshooter.voltage = Volts.mutable(0);

    // shooterMotors = List.of(leftMotor, middleMotor, rightMotor);

    // configureShooterMotor(leftMotor, InvertedValue.CounterClockwise_Positive);
    // configureShooterMotor(middleMotor, InvertedValue.Clockwise_Positive);
    // configureShooterMotor(rightMotor, InvertedValue.Clockwise_Positive);

    // configureFeederMotor(feederMotor);
  }

  // private void configureShooterMotor(TalonFX motor, InvertedValue inversion) {

  //   TalonFXConfiguration config =
  //       new TalonFXConfiguration()
  //           .withMotorOutput(
  //               new MotorOutputConfigs()
  //                   .withInverted(inversion)
  //                   .withNeutralMode(NeutralModeValue.Coast))
  //           .withCurrentLimits(
  //               new CurrentLimitsConfigs()
  //                   .withStatorCurrentLimit(80)
  //                   .withStatorCurrentLimitEnable(true)
  //                   .withSupplyCurrentLimit(40)
  //                   .withSupplyCurrentLimitEnable(true));

  //   motor.getConfigurator().apply(config);
  // }

  // private void configureFeederMotor(TalonFX motor) {

  //   TalonFXConfiguration config =
  //       new TalonFXConfiguration()
  //           .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
  //           .withCurrentLimits(
  //               new CurrentLimitsConfigs()
  //                   .withStatorCurrentLimit(60)
  //                   .withStatorCurrentLimitEnable(true)
  //                   .withSupplyCurrentLimit(30)
  //                   .withSupplyCurrentLimitEnable(true));

  //   motor.getConfigurator().apply(config);
  // }

  // Flywheel
  // public void runShooter() {
  //   for (TalonFX motor : shooterMotors) {
  //     motor.setControl(voltageRequest.withOutput(SHOOTER_SPEED * 12.0));
  //   }
  // }

  public void setTarget(Voltage target) {
    m_ShooterIO.setTarget(target);
  }

  public Command getNewSetVoltsCommand(LoggedTunableNumber volts) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of(volts.get()));
        },
        this);
  }

  public Command getNewSetVoltsCommand(double i) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of(i));
        },
        this);
  }

  @Override
  public void periodic() {
    m_ShooterIO.updateInputs(loggedshooter);
  }

  // public void stopShooter() {
  //   for (TalonFX motor : shooterMotors) {
  //     motor.setControl(voltageRequest.withOutput(0.0));
  //   }
  // }

  // // Feeder

  // public void runFeeder() {
  //   feederMotor.setControl(voltageRequest.withOutput(-(FEEDER_SPEED * 12.0)));
  // }

  // public void runFeederReverse() {
  //   feederMotor.setControl(voltageRequest.withOutput((FEEDER_SPEED * 12.0)));
  // }

  // public void stopFeeder() {
  //   feederMotor.setControl(voltageRequest.withOutput(0.0));
  // }

  // public void stopAll() {
  //   stopShooter();
  //   stopFeeder();
  // }
}
