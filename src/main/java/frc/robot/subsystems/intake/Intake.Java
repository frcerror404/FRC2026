package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final TalonFX intakeMotor;

  private static final double INTAKE_SPEED = 0.7;
  private static final double OUTTAKE_SPEED = -0.7;

  public Intake(int canId) {
    intakeMotor = new TalonFX(canId);
  }

  public void intakeIn() {
    intakeMotor.setControl(new DutyCycleOut(INTAKE_SPEED));
  }

  public void intakeOut() {
    intakeMotor.setControl(new DutyCycleOut(OUTTAKE_SPEED));
  }

  public void stop() {
    intakeMotor.setControl(new DutyCycleOut(0.0));
  }
}
