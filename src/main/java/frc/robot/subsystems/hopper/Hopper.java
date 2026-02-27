package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

 
  private static final double HOPPER_SPEED = 0.7;

  private final TalonFX hopperMotor;

  private final VoltageOut rollerRequest = new VoltageOut(0);
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

  // CAN IDs
  public Hopper(int hopperID) {
    hopperMotor = new TalonFX(hopperID);

    configureHopper();

  }

  private void configureHopper() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(60)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(35)
                    .withSupplyCurrentLimitEnable(true));

    hopperMotor.getConfigurator().apply(config);
  }

  
  // Roller Control
  public void runHopper() {
    hopperMotor.setControl(new DutyCycleOut(HOPPER_SPEED));
  }


  public void stopHopper() {
    hopperMotor.setControl(new DutyCycleOut(0.0));
  }

  // 
}
