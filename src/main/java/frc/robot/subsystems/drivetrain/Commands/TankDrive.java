package frc.robot.subsystems.drivetrain.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class TankDrive extends Command {
    private Drivetrain drivetrain;
    private Supplier<Double> lSpeedSupplier;
    private Supplier<Double> rSpeedSupplier;

    /**
     * Creates a new TankDrive command.
     * This command drives a {@link Drivetrain} using open loop controls.
     * It drives with one joystick controlling the speed of the left side of the robot, and another joystick controlling the right side of the robot.
     *
     * @param lSpeedSupplier The supplier for the left speed.
     * @param rSpeedSupplier The supplier for the right speed.
     */
    public TankDrive(Drivetrain drivetrain, Supplier<Double> lSpeedSupplier, Supplier<Double> rSpeedSupplier) {
        this.drivetrain = drivetrain;
        this.lSpeedSupplier = lSpeedSupplier;
        this.rSpeedSupplier = rSpeedSupplier;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    /** Called every time the scheduler runs while this command is scheduled. */
    @Override
    public void execute() {
        // Executing the suppliers
        double lSpeed = MathUtils.applyDeadbandWithOffsets(lSpeedSupplier.get(), Constants.deadband, 0.9);
        double rSpeed = MathUtils.applyDeadbandWithOffsets(rSpeedSupplier.get(), Constants.deadband, 0.9);

        // Scaling for max speeds
        lSpeed *= DriveConstants.maxOpenDriveSpeed;
        rSpeed *= DriveConstants.maxOpenTurnSpeed;

        // Driving the robot
        drivetrain.tankDrive(lSpeed, rSpeed);
    }

    /** Called when the command is cancelled, either by the scheduler or when {@link TankDrive#isFinished} returns true. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDrive(0, 0);
    }
}