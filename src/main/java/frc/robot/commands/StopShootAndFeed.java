package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class StopShootAndFeed extends SequentialCommandGroup {

  public StopShootAndFeed(
      Hopper hopper,
      Feeder feeder1,
      Feeder feeder2,
      Shooter shooter1,
      Shooter shooter2,
      Shooter shooter3,
      Shooter shooter4) {
    super(
        hopper.getStopCommand(),
        feeder1.getStopCommand(),
        feeder2.getStopCommand(),
        shooter1.getStopCommand(),
        shooter2.getStopCommand(),
        shooter3.getStopCommand(),
        shooter4.getStopCommand());
    addRequirements(hopper, feeder1, feeder2, shooter1, shooter2, shooter3, shooter4);
  }
}
