package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final TalonFX shooterMotor0;
  private final TalonFX shooterMotor1;
  private final TalonFX shooterMotor2;

  private static final double SHOOTER_SPEED = 0.7;

  public Shooter(int canId) {
    shooterMotor0 = new TalonFX(canId);
    shooterMotor1 = new TalonFX(canId);
    shooterMotor2 = new TalonFX(canId);
  }

  public void shooterOut() {
    shooterMotor0.setControl(new DutyCycleOut(SHOOTER_SPEED));
    shooterMotor1.setControl(new DutyCycleOut(SHOOTER_SPEED));
    shooterMotor2.setControl(new DutyCycleOut(SHOOTER_SPEED));
  }

  public void stop() {
    shooterMotor0.setControl(new DutyCycleOut(0.0));
    shooterMotor1.setControl(new DutyCycleOut(0.0));
    shooterMotor2.setControl(new DutyCycleOut(0.0));
  }
}
