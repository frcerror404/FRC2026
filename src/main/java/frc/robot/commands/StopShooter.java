package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterReverse.ShooterReverse;

public class StopShooter extends SequentialCommandGroup {

  public StopShooter(ShooterReverse shooter1, Shooter shooter2, Shooter shooter3) {
    super(shooter1.getStopCommand(), shooter2.getStopCommand(), shooter3.getStopCommand());
    addRequirements(shooter1, shooter2, shooter3);
  }
}
