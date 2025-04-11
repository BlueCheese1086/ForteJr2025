package frc.robot.subsystems.drivetrain.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class ArcadeDrive extends Command {
    private Drivetrain drivetrain;
    private Supplier<Double> xSpeedSupplier;
    private Supplier<Double> zRotatSupplier;

    /**
     * Creates a new {@link ArcadeDrive} command.
     * This command drives a {@link Drivetrain} using open loop controls.
     * It drives with one joystick controlling the forwards/backwards speeds of the robot, and another joystick controlling the turn speeds of the robot.
     *
     * @param xSpeedSupplier The supplier for the x translational speed.
     * @param zRotateSupplier The supplier for the z rotational speed.
     */
    public ArcadeDrive(Drivetrain drivetrain, Supplier<Double> xSpeedSupplier, Supplier<Double> zRotateSupplier) {
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.zRotatSupplier = zRotateSupplier;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    /** Called every time the scheduler runs while this command is scheduled. */
    @Override
    public void execute() {
        // Executing the suppliers
        double xSpeed = xSpeedSupplier.get();
        double zRotat = zRotatSupplier.get();

        // Applying a deadband
        MathUtils.applyDeadbandWithOffsets(xSpeed, Constants.deadband, 0.9);
        MathUtils.applyDeadbandWithOffsets(zRotat, Constants.deadband, 0.9);

        // Scaling for max speeds
        xSpeed *= DriveConstants.maxDrivePercent;
        zRotat *= DriveConstants.maxSteerPercent;

        // Driving the robot
        drivetrain.arcadeDrive(xSpeed, zRotat);
    }

    /** Called when the command is cancelled, either by the scheduler or when {@link ArcadeDrive#isFinished} returns true. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
    }
}