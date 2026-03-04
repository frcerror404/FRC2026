package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final ClimberIO m_ClimberIO;

  ClimberIOInputsAutoLogged loggedclimber = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO climberIO) {
    m_ClimberIO = climberIO;
    loggedclimber.angularVelocity = DegreesPerSecond.mutable(0);
    loggedclimber.supplyCurrent = Amps.mutable(0);
    loggedclimber.torqueCurrent = Amps.mutable(0);
    loggedclimber.voltage = Volts.mutable(0);
  }

  public Command runClimber(double climberSpeed) {
    return new InstantCommand(() -> m_ClimberIO.runClimber(climberSpeed), this);
  }

  public Command getStopCommand() {
    return new InstantCommand(() -> m_ClimberIO.stop(), this);
  }

  @Override
  public void periodic() {
    m_ClimberIO.updateInputs(loggedclimber);
  }
}
