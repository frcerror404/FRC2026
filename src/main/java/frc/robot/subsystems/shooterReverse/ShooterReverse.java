package frc.robot.subsystems.shooterReverse;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class ShooterReverse extends SubsystemBase {
  private final ShooterReverseIO m_ShooterIO;

  ShooterReverseIOInputsAutoLogged loggedshooterReverse = new ShooterReverseIOInputsAutoLogged();

  public ShooterReverse(ShooterReverseIO ShooterIO) {
    m_ShooterIO = ShooterIO;
    loggedshooterReverse.angularVelocity = DegreesPerSecond.mutable(0);
    loggedshooterReverse.supplyCurrent = Amps.mutable(0);
    loggedshooterReverse.torqueCurrent = Amps.mutable(0);
    loggedshooterReverse.voltageSetPoint = Volts.mutable(0);
    loggedshooterReverse.voltage = Volts.mutable(0);
  }

  public void setTarget(Voltage target) {
    m_ShooterIO.setTarget(target);
  }

  public Command shootFuel(double shotSpeed) {
    return new InstantCommand(() -> m_ShooterIO.shootFuel(shotSpeed), this);
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
    return new InstantCommand(() -> m_ShooterIO.stop(), this);
  }

  @Override
  public void periodic() {
    m_ShooterIO.updateInputs(loggedshooterReverse);
  }
}
