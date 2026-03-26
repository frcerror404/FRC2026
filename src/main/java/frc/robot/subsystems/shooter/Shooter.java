package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableGainsBuilder;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO m_ShooterIO;

  ShooterIOInputsAutoLogged loggedshooter = new ShooterIOInputsAutoLogged();

  public LoggedTunableGainsBuilder tunableGains =
      new LoggedTunableGainsBuilder("Shooter", 35, 0, 1, 0, .288, 0, 0, 300, 10, 0, 0, 0);

  public Shooter(ShooterIO ShooterIO) {
    m_ShooterIO = ShooterIO;
    loggedshooter.shootermotor1supplyCurrent = Amps.mutable(0);
    loggedshooter.shootermotor1statorCurrent = Amps.mutable(0);
    loggedshooter.shootermotor1torqueCurrent = Amps.mutable(0);
    loggedshooter.shootermotor1voltage = Volts.mutable(0);
    loggedshooter.shootermotor1velocity = RotationsPerSecond.mutable(0);
    loggedshooter.shootermotor1Temp = Celsius.mutable(0);
    loggedshooter.shootermotor2supplyCurrent = Amps.mutable(0);
    loggedshooter.shootermotor2statorCurrent = Amps.mutable(0);
    loggedshooter.shootermotor2torqueCurrent = Amps.mutable(0);
    loggedshooter.shootermotor2voltage = Volts.mutable(0);
    loggedshooter.shootermotor2velocity = RotationsPerSecond.mutable(0);
    loggedshooter.shootermotor2Temp = Celsius.mutable(0);
    loggedshooter.shootermotor3supplyCurrent = Amps.mutable(0);
    loggedshooter.shootermotor3statorCurrent = Amps.mutable(0);
    loggedshooter.shootermotor3torqueCurrent = Amps.mutable(0);
    loggedshooter.shootermotor3voltage = Volts.mutable(0);
    loggedshooter.shootermotor3velocity = RotationsPerSecond.mutable(0);
    loggedshooter.shootermotor3Temp = Celsius.mutable(0);
    loggedshooter.shootermotor4supplyCurrent = Amps.mutable(0);
    loggedshooter.shootermotor4statorCurrent = Amps.mutable(0);
    loggedshooter.shootermotor4torqueCurrent = Amps.mutable(0);
    loggedshooter.shootermotor4voltage = Volts.mutable(0);
    loggedshooter.shootermotor4velocity = RotationsPerSecond.mutable(0);
    loggedshooter.shootermotor4Temp = Celsius.mutable(0);

    this.m_ShooterIO.shooterPID(tunableGains.build());
  }

  public Command shootFuel(double shotSpeed) {
    return new InstantCommand(() -> m_ShooterIO.shootFuel(shotSpeed), this);
  }

  public Command getStopCommand() {
    return new InstantCommand(() -> m_ShooterIO.stop(), this);
  }

  @Override
  public void periodic() {
    tunableGains.ifGainsHaveChanged((gains) -> this.m_ShooterIO.shooterPID(gains));
    m_ShooterIO.updateInputs(loggedshooter);
    Logger.processInputs("RobotState/Shooter", loggedshooter);
  }
}
