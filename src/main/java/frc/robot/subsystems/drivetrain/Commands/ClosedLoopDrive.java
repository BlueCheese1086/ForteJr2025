package frc.robot.subsystems.drivetrain.Commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class ClosedLoopDrive extends Command {
    private Drivetrain drivetrain;
    private Supplier<Double> xSpeedSupplier;
    private Supplier<Double> zRotatSupplier;

    /**
     * Creates a new {@link ClosedLoopDrive} command.
     * This command drives a {@link Drivetrain} using a closed-loop PID controller.
     * 
     * @param xSpeedSupplier The supplier for the x translational speed.
     * @param zRotateSupplier The supplier for the z rotational speed.
     */
    public ClosedLoopDrive(Drivetrain drivetrain, Supplier<Double> xSpeedSupplier, Supplier<Double> zRotateSupplier) {
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.zRotatSupplier = zRotateSupplier;

        // Preventing the robot from being driven by multiple commands.
        addRequirements(drivetrain);
    }

    /** Called every time the scheduler runs while this command is scheduled. */
    @Override
    public void execute() {
        // Executing the suppliers
        double xSpeed = MathUtils.applyDeadbandWithOffsets(xSpeedSupplier.get(), Constants.deadband, 0.9);
        double zRotat = MathUtils.applyDeadbandWithOffsets(zRotatSupplier.get(), Constants.deadband, 0.9);

        // Scaling for max speeds
        xSpeed *= DriveConstants.maxDriveSpeed.in(MetersPerSecond);
        zRotat *= DriveConstants.maxSteerSpeed.in(MetersPerSecond);
        
        // Driving the robot
        drivetrain.closedLoop(new ChassisSpeeds(xSpeed, 0, zRotat));
    }

    /** Called when the command is cancelled, either by the scheduler or when {@link ClosedLoopDrive#isFinished} returns true. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.closedLoop(new ChassisSpeeds());
    }
}