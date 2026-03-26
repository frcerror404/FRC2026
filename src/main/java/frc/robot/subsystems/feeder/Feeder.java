package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO m_FeederIO;

  FeederIOInputsAutoLogged loggedfeeder = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO FeederIO) {
    m_FeederIO = FeederIO;
    loggedfeeder.feedermotor1supplyCurrent = Amps.mutable(0);
    loggedfeeder.feedermotor1statorCurrent = Amps.mutable(0);
    loggedfeeder.feedermotor1torqueCurrent = Amps.mutable(0);
    loggedfeeder.feedermotor1voltage = Volts.mutable(0);
    loggedfeeder.feedermotor1velocity = RotationsPerSecond.mutable(0);
    loggedfeeder.feedermotor1Temp = Celsius.mutable(0);
    loggedfeeder.feedermotor2supplyCurrent = Amps.mutable(0);
    loggedfeeder.feedermotor2statorCurrent = Amps.mutable(0);
    loggedfeeder.feedermotor2torqueCurrent = Amps.mutable(0);
    loggedfeeder.feedermotor2voltage = Volts.mutable(0);
    loggedfeeder.feedermotor2velocity = RotationsPerSecond.mutable(0);
    loggedfeeder.feedermotor2Temp = Celsius.mutable(0);
  }

  public Command runFeeder(double feederSpeed) {
    return new InstantCommand(() -> m_FeederIO.runFeeder(feederSpeed), this);
  }

  public Command getStopCommand() {
    return new InstantCommand(() -> m_FeederIO.stop(), this);
  }

  @Override
  public void periodic() {
    m_FeederIO.updateInputs(loggedfeeder);
    Logger.processInputs("RobotState/Feeder", loggedfeeder);
  }
}
