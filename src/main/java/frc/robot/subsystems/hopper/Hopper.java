package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class Hopper extends SubsystemBase {
  private final HopperIO m_HopperIO;

  HopperIOInputsAutoLogged loggedhopper = new HopperIOInputsAutoLogged();

  public Hopper(HopperIO hopperIO) {
    m_HopperIO = hopperIO;
    loggedhopper.angularVelocity = DegreesPerSecond.mutable(0);
    loggedhopper.supplyCurrent = Amps.mutable(0);
    loggedhopper.torqueCurrent = Amps.mutable(0);
    loggedhopper.voltageSetPoint = Volts.mutable(0);
    loggedhopper.voltage = Volts.mutable(0);
  }

  public Command getNewSetVoltsCommand(DoubleSupplier volts) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of((volts.getAsDouble())));
        },
        this);
  }

  public void setTarget(Voltage target) {
    m_HopperIO.setTarget(target);
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
    return new InstantCommand(() -> m_HopperIO.stop(), this);
  }

  @Override
  public void periodic() {
    m_HopperIO.updateInputs(loggedhopper);
  }
}
