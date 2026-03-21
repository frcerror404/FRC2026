package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class StopShootAndFeed extends SequentialCommandGroup {

  public StopShootAndFeed(Hopper hopper, Feeder feeder, Shooter shooter) {
    super(hopper.getStopCommand(), feeder.getStopCommand(), shooter.getStopCommand());
    addRequirements(hopper, feeder, shooter);
  }
}
