package frc.robot.subsystems.drivetrain.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.AdjustableValues;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class ArcadeDrive extends Command {
    private Drivetrain drivetrain;
    private Supplier<Double> xSpeedSupplier;
    private Supplier<Double> zSteerSupplier;

    /**
     * Creates a new {@link ArcadeDrive} command.
     * This command drives a {@link Drivetrain} using open loop controls.
     * It drives with one joystick controlling the forwards/backwards speed of the robot, and another joystick controlling the turn speed of the robot.
     *
     * @param xSpeedSupplier The supplier for the x translational speed.
     * @param zSteerSupplier The supplier for the z rotational speed.
     */
    public ArcadeDrive(Drivetrain drivetrain, Supplier<Double> xSpeedSupplier, Supplier<Double> zSteerSupplier) {
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.zSteerSupplier = zSteerSupplier;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    /** Called every time the scheduler runs while this command is scheduled. */
    @Override
    public void execute() {
        // Executing the suppliers
        double xSpeed = -xSpeedSupplier.get();
        double zSteer =  zSteerSupplier.get();

        // Applying a deadband
        MathUtils.applyDeadbandWithOffsets(xSpeed, Constants.deadband, 0.9);
        MathUtils.applyDeadbandWithOffsets(zSteer, Constants.deadband, 0.9);

        // Scaling for max speeds
        xSpeed *= AdjustableValues.getNumber("Drive_Percent");
        zSteer *= AdjustableValues.getNumber("Steer_Percent");

        // Driving the robot
        drivetrain.arcadeDrive(xSpeed, zSteer);
    }

    /** Called when the command is cancelled, either by the scheduler or when {@link ArcadeDrive#isFinished} returns true. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
    }
}