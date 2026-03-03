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
  private final ShooterIO m_ShooterIO;

  ShooterIOInputsAutoLogged loggedshooter = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO ShooterIO) {
    m_ShooterIO = ShooterIO;
    loggedshooter.angularVelocity = DegreesPerSecond.mutable(0);
    loggedshooter.supplyCurrent = Amps.mutable(0);
    loggedshooter.torqueCurrent = Amps.mutable(0);
    loggedshooter.voltageSetPoint = Volts.mutable(0);
    loggedshooter.voltage = Volts.mutable(0);
  }

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

  public Command shootFuel(Double shotSpeed) {
    return new InstantCommand(() -> m_ShooterIO.shootFuel(shotSpeed), this);
  }

  public Command getNewSetVoltsCommand(double i) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of(i));
        },
        this);
  }

  public Command getStopCommand() {
    return new InstantCommand(() -> m_ShooterIO.stop(), this);
  }

  @Override
  public void periodic() {
    m_ShooterIO.updateInputs(loggedshooter);
  }
}
