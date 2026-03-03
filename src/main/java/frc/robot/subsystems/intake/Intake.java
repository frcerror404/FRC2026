package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
  private final IntakeIO m_IntakeIO;

  IntakeIO.IntakeIOInputs loggedintake = new IntakeIO.IntakeIOInputs();

  public Intake(IntakeIO IntakeIO) {
    m_IntakeIO = IntakeIO;
    loggedintake.angularVelocity = DegreesPerSecond.mutable(0);
    loggedintake.supplyCurrent = Amps.mutable(0);
    loggedintake.torqueCurrent = Amps.mutable(0);
    loggedintake.voltageSetPoint = Volts.mutable(0);
    loggedintake.voltage = Volts.mutable(0);
  }

  public void setTarget(Voltage target) {
    m_IntakeIO.setTarget(target);
  }

  public Command getNewSetVoltsCommand(DoubleSupplier volts) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of((volts.getAsDouble())));
        },
        this);
  }

  public Command runIntake(double intakeSpeed) {
    return new InstantCommand(() -> m_IntakeIO.runIntake(intakeSpeed), this);
  }

  public Command runIntakeReverse(double intakeSpeed) {
    return new InstantCommand(() -> m_IntakeIO.runIntakeReverse(intakeSpeed), this);
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

  public Command getStopCommand() {
    return new InstantCommand(() -> m_IntakeIO.stop(), this);
  }

  @Override
  public void periodic() {
    m_IntakeIO.updateInputs(loggedintake);
  }
}
