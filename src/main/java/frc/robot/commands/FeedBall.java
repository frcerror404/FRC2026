// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.shooter.Shooter;

// public class FeedBall extends Command {

//   private final Shooter shooter;

//   public FeedBall(Shooter shooter) {
//     this.shooter = shooter;
//     addRequirements(shooter);
//   }

//   @Override
//   public void initialize() {
//     shooter.runFeeder();
//   }

//   @Override
//   public void end(boolean interrupted) {
//     shooter.stopFeeder();
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
