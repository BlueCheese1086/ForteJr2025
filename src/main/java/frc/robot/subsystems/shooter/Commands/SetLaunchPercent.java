package frc.robot.subsystems.shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class SetLaunchPercent extends Command {
    private Shooter shooter;
    private double speed;

    public SetLaunchPercent(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
    }

    @Override
    public void execute() {
        shooter.setLaunchPercent(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setLaunchPercent(0);
    }
}