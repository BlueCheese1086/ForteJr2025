package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RecordPose extends Command {
    private Supplier<Pose2d> poseSupplier;
    private int x = 0;

    /**
     * Creates a new {@link RecordPose} command.
     * 
     * @param poseSupplier The function to recieve poses from.
     */
    public RecordPose(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {
        Logger.recordOutput("/RecordedPoses/" + x, poseSupplier.get());
        x += 1;
        cancel();
    }
}