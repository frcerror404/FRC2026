package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO m_HopperIO;

  HopperIOInputsAutoLogged loggedhopper = new HopperIOInputsAutoLogged();

  public Hopper(HopperIO hopperIO) {
    m_HopperIO = hopperIO;
    loggedhopper.hoppervelocity = DegreesPerSecond.mutable(0);
    loggedhopper.hoppersupplyCurrent = Amps.mutable(0);
    loggedhopper.hopperstatorCurrent = Amps.mutable(0);
    loggedhopper.hoppertorqueCurrent = Amps.mutable(0);
    loggedhopper.hoppervoltageSetPoint = Volts.mutable(0);
    loggedhopper.hoppervoltage = Volts.mutable(0);
    loggedhopper.hoppervelocity = RotationsPerSecond.mutable(0);
    loggedhopper.hoppertemperature = Celsius.mutable(0);
  }

  public Command runHopper(double hopperSpeed) {
    return new InstantCommand(() -> m_HopperIO.runHopper(hopperSpeed), this);
  }

  public Command getStopCommand() {
    return new InstantCommand(() -> m_HopperIO.stop(), this);
  }

  @Override
  public void periodic() {
    m_HopperIO.updateInputs(loggedhopper);
    Logger.processInputs("RobotState/Hopper", loggedhopper);
  }
}
