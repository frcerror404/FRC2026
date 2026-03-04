package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final IntakeIO m_IntakeIO;

  IntakeIO.IntakeIOInputs loggedintake = new IntakeIO.IntakeIOInputs();

  public Intake(IntakeIO IntakeIO) {
    m_IntakeIO = IntakeIO;
    loggedintake.angularVelocity = DegreesPerSecond.mutable(0);
    loggedintake.supplyCurrent = Amps.mutable(0);
    loggedintake.torqueCurrent = Amps.mutable(0);
    loggedintake.voltage = Volts.mutable(0);
  }

  public Command runIntake(double intakeSpeed) {
    return new InstantCommand(() -> m_IntakeIO.runIntake(intakeSpeed), this);
  }

  public Command getStopCommand() {
    return new InstantCommand(() -> m_IntakeIO.stop(), this);
  }

  @Override
  public void periodic() {
    m_IntakeIO.updateInputs(loggedintake);
  }
}
