package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class Feeder extends SubsystemBase {
  private final FeederIO m_FeederIO;

  FeederIOInputsAutoLogged loggedfeeder = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO FeederIO) {
    m_FeederIO = FeederIO;
    loggedfeeder.angularVelocity = DegreesPerSecond.mutable(0);
    loggedfeeder.supplyCurrent = Amps.mutable(0);
    loggedfeeder.torqueCurrent = Amps.mutable(0);
    loggedfeeder.voltageSetPoint = Volts.mutable(0);
    loggedfeeder.voltage = Volts.mutable(0);
  }

  public Command getNewSetVoltsCommand(DoubleSupplier volts) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of((volts.getAsDouble())));
        },
        this);
  }

  public void setTarget(Voltage target) {
    m_FeederIO.setTarget(target);
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
    return new InstantCommand(() -> m_FeederIO.stop(), this);
  }

  @Override
  public void periodic() {
    m_FeederIO.updateInputs(loggedfeeder);
  }
}
