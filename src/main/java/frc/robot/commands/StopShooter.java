package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

public class StopShooter extends SequentialCommandGroup {

  public StopShooter(Shooter shooter1, Shooter shooter2, Shooter shooter3, Shooter shooter4) {
    super(
        shooter1.getStopCommand(),
        shooter2.getStopCommand(),
        shooter3.getStopCommand(),
        shooter4.getStopCommand());
    addRequirements(shooter1, shooter2, shooter3, shooter4);
  }
}
