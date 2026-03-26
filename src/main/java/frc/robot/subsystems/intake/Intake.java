package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO m_IntakeIO;

  IntakeIOInputsAutoLogged loggedintake = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO IntakeIO) {
    m_IntakeIO = IntakeIO;
    loggedintake.intakemotor1supplyCurrent = Amps.mutable(0);
    loggedintake.intakemotor1statorCurrent = Amps.mutable(0);
    loggedintake.intakemotor1torqueCurrent = Amps.mutable(0);
    loggedintake.intakemotor1voltage = Volts.mutable(0);
    loggedintake.intakemotor1velocity = RotationsPerSecond.mutable(0);
    loggedintake.intakemotor1Temp = Celsius.mutable(0);
    loggedintake.intakemotor2supplyCurrent = Amps.mutable(0);
    loggedintake.intakemotor2statorCurrent = Amps.mutable(0);
    loggedintake.intakemotor2torqueCurrent = Amps.mutable(0);
    loggedintake.intakemotor2voltage = Volts.mutable(0);
    loggedintake.intakemotor2velocity = RotationsPerSecond.mutable(0);
    loggedintake.intakemotor2Temp = Celsius.mutable(0);
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
    Logger.processInputs("RobotState/Intake", loggedintake);
  }
}
