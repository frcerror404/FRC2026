package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO m_ShooterIO;

  ShooterIOInputsAutoLogged loggedshooter = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO ShooterIO) {
    m_ShooterIO = ShooterIO;
    loggedshooter.supplyCurrent = Amps.mutable(0);
    loggedshooter.torqueCurrent = Amps.mutable(0);
    loggedshooter.voltage = Volts.mutable(0);
  }

  public Command shootFuel(double shotSpeed, boolean isReverse) {
    return new InstantCommand(() -> m_ShooterIO.shootFuel(shotSpeed, isReverse), this);
  }

  public Command getStopCommand() {
    return new InstantCommand(() -> m_ShooterIO.stop(), this);
  }

  @Override
  public void periodic() {
    m_ShooterIO.updateInputs(loggedshooter);
  }
}
