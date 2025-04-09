package frc.robot.subsystems.shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class SetFeedPercent extends Command {
    private Shooter shooter;
    private double speed;

    public SetFeedPercent(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
    }

    @Override
    public void execute() {
        shooter.setFeedPercent(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFeedPercent(0);
    }
}