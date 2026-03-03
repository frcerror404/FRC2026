package frc.robot.subsystems.shooterReverse;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterReverse extends SubsystemBase {
  private final ShooterReverseIO m_ShooterIO;

  ShooterReverseIOInputsAutoLogged loggedshooterReverse = new ShooterReverseIOInputsAutoLogged();

  public ShooterReverse(ShooterReverseIO ShooterIO) {
    m_ShooterIO = ShooterIO;
    loggedshooterReverse.supplyCurrent = Amps.mutable(0);
    loggedshooterReverse.torqueCurrent = Amps.mutable(0);
    loggedshooterReverse.voltage = Volts.mutable(0);
  }

  public Command shootFuel(double shotSpeed) {
    return new InstantCommand(() -> m_ShooterIO.shootFuel(shotSpeed), this);
  }

  public Command getStopCommand() {
    return new InstantCommand(() -> m_ShooterIO.stop(), this);
  }

  @Override
  public void periodic() {
    m_ShooterIO.updateInputs(loggedshooterReverse);
  }
}
