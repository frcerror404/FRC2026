package frc.robot.subsystems.intakeAngle;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.RobotState;
import frc.robot.util.LoggedTunableGainsBuilder;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class IntakeAngle extends SubsystemBase {
  private IntakeAngleIO m_IntakeAngleIO;

  IntakeAngleIOInputsAutoLogged loggedintakeangle = new IntakeAngleIOInputsAutoLogged();

  public LoggedTunableGainsBuilder tunableGains =
      new LoggedTunableGainsBuilder("IntakeAngle", 35, 0, 1, 0, .288, 0, 0, 300, 10, 0, 0, 0);

  public IntakeAngle(IntakeAngleIO intakeAngleIO) {
    m_IntakeAngleIO = intakeAngleIO;
    loggedintakeangle.intakeAngle = Degrees.mutable(0);
    loggedintakeangle.intakeAngularVelocity = DegreesPerSecond.mutable(0);
    loggedintakeangle.intakeAngleSetPoint = Degrees.mutable(0);
    loggedintakeangle.supplyCurrent = Amps.mutable(0);
    loggedintakeangle.torqueCurrent = Amps.mutable(0);
    loggedintakeangle.voltageSetPoint = Volts.mutable(0);
    loggedintakeangle.voltage = Volts.mutable(0);

    this.m_IntakeAngleIO.setGains(tunableGains.build());

    // RobotState.instance().setClawAngleSource(loggedclawangle.clawAngle);
  }

  public Supplier<Angle> getAngleSupplier() {
    return () -> loggedintakeangle.intakeAngle;
  }

  public void setAngle(Angle angle) {
    m_IntakeAngleIO.setTarget(angle);
  }

  public Command getNewIntakeAngleTurnCommand(DoubleSupplier angle) {
    return new InstantCommand(
        () -> {
          setAngle(Degrees.of(angle.getAsDouble()));
        },
        this);
  }

  public Command getNewApplyCoastModeCommand() {
    return new InstantCommand(
        () -> {
          m_IntakeAngleIO.applyCoastMode();
        },
        this);
  }

  public Command getNewIntakeAngleTurnCommand(double i) {
    return new InstantCommand(
        () -> {
          setAngle(Degrees.of(i));
        },
        this);
  }

  /**
   * Returns when this joint is greater than 'angle' away from the forward horizontal
   *
   * @param angle
   * @return
   */
  public Trigger getNewGreaterThanAngleTrigger(DoubleSupplier angle) {
    return new Trigger(
        () -> {
          return loggedintakeangle.intakeAngle.in(Degrees) > angle.getAsDouble();
        });
  }

  public Trigger getNewAtAngleTrigger(Angle angle, Angle tolerance) {
    return new Trigger(
        () -> {
          return MathUtil.isNear(
              angle.baseUnitMagnitude(),
              loggedintakeangle.intakeAngle.baseUnitMagnitude(),
              tolerance.baseUnitMagnitude());
        });
  }

  public Trigger getNewAtSetpointTrigger() {
    return new Trigger(
        () -> {
          return MathUtil.isNear(
              loggedintakeangle.intakeAngleSetPoint.baseUnitMagnitude(),
              loggedintakeangle.intakeAngle.baseUnitMagnitude(),
              Degrees.of(0.25).baseUnitMagnitude());
        });
  }

  public boolean isIntakeAngleAtAngle(Angle angle, Angle tolerance) {
    return MathUtil.isNear(
        angle.baseUnitMagnitude(),
        loggedintakeangle.intakeAngle.baseUnitMagnitude(),
        tolerance.baseUnitMagnitude());
  }

  @Override
  public void periodic() {
    tunableGains.ifGainsHaveChanged((gains) -> this.m_IntakeAngleIO.setGains(gains));
    m_IntakeAngleIO.updateInputs(loggedintakeangle);
    Logger.processInputs("RobotState/IntakeAngle", loggedintakeangle);
  }
}
