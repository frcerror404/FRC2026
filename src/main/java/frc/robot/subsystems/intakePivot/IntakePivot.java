package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotIO m_IntakePivotIO;

  IntakePivotIO.IntakePivotIOInputs loggedintakepivot = new IntakePivotIO.IntakePivotIOInputs();
  // CAN IDs
  public IntakePivot(IntakePivotIO intakePivotIO) {
    m_IntakePivotIO = intakePivotIO;
    loggedintakepivot.angularVelocity = DegreesPerSecond.mutable(0);
    loggedintakepivot.supplyCurrent = Amps.mutable(0);
    loggedintakepivot.torqueCurrent = Amps.mutable(0);
    loggedintakepivot.voltage = Volts.mutable(0);
    loggedintakepivot.intakeAngle = Degrees.mutable(0);
  }

  public void stop() {
    m_IntakePivotIO.stop();
  }

  public void setBrakeMode(boolean Enabled) {
    m_IntakePivotIO.setBrakeMode(Enabled);
  }

  public Command pivotToStow() {
    return m_IntakePivotIO.pivotToStow();
  }

  public Command pivotToIntake() {
    return m_IntakePivotIO.pivotToIntake();
  }
}
