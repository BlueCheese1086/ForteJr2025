package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs;

    public Shooter(ShooterIO io) {
        this.io = io;

        inputs = new ShooterIOInputsAutoLogged();
    }

    /** This function runs every tick that the class has been initialized. */
    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("/RealOutputs/Shooter", inputs);
    }

    /** Gets the speed of the feed wheel. */
    public double getFeedSpeed() {
        return inputs.feedPercent;
    }

    /** Gets the speed of the launch wheel. */
    public double getLaunchSpeed() {
        return inputs.launchPercent;
    }

    /** Sets the speed of the feed wheel. */
    public void setFeedPercent(double speed) {
        Logger.recordOutput("/Shooter/Expected_Feed_Speed", speed);

        io.setFeedPercent(speed);
    }

    /** Sets the speed of the launch wheel. */
    public void setLaunchPercent(double speed) {
        Logger.recordOutput("/Shooter/Expected_Launch_Speed", speed);

        io.setFeedPercent(speed);
    }
}