package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

public class StopShooter extends SequentialCommandGroup {

  public StopShooter(Shooter shooter, Shooter shooter2, Shooter shooter3) {
    super(shooter.getStopCommand(), shooter2.getStopCommand(), shooter3.getStopCommand());
    addRequirements(shooter, shooter2, shooter3);
  }
}
