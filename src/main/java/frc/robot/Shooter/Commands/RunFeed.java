package frc.robot.Shooter.Commands;

import frc.robot.Shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class RunFeed extends Command {
    private Shooter shooter;
    private double speed;

    public RunFeed(double speed) {
        this.shooter = Shooter.getInstance();
        this.speed = speed;
    }

    @Override
    public void execute() {
        shooter.setFeedSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFeedSpeed(0);
    }
}