package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class MathUtils {
    /**
     * Applies a deadband and offsets the output by the deadband.
     * This allows you to still reach that lower range you would normally lose when using a normal deadband.
     * 
     * @param value The number to apply a deadband to.
     * @param deadband The deadband.
     */
    public static double applyDeadbandWithOffsets(double value, double deadband) {
        if (Math.abs(value) < Math.abs(deadband)) return 0;

        return value - Math.copySign(deadband, value);
    }

    /**
     * Applies a deadband and offsets the output by the deadband.
     * This allows you to still reach that lower range you would normally lose when using a normal deadband.
     * However, it also allows you to run at the higher range output that would be lost by the offset.
     * Once the value exceeds vanillaVal, the offset is no longer applied to it.
     * 
     * @param value The number to apply a deadband to.
     * @param deadband The deadband.
     * @param vanillaVal The value where the offset is no longer applied.
     */
    public static double applyDeadbandWithOffsets(double value, double deadband, double vanillaVal) {
        if (Math.abs(value) < Math.abs(deadband)) return 0;

        if (Math.abs(value) >= Math.abs(vanillaVal)) return value;

        return value - Math.copySign(deadband, value);
    }

    /**
     * Checks if a number is within a deadband.
     * 
     * @param value The number to check.
     * @param deadband The deadband.
     */
    public static boolean withinDeadband(double value, double deadband) {
        return Math.abs(value) < Math.abs(deadband);
    }

    /**
     * Finds a {@link Translation2d} between all of the parameters.
     * @param translations The translations to analyze
     */
    public Translation2d getMean(Translation2d... translations) {
        if (translations.length == 0) return new Translation2d();
        if (translations.length == 1) return translations[0];

        Translation2d interpolated = translations[0].interpolate(translations[1], 0.5);
        double done = 2;
        double total = 2;
        for (int i = 2; i < translations.length; i++) {
            total++;
            interpolated = translations[i].interpolate(interpolated, done / total);
            done++;
        }

        return interpolated;
    }

    /**
     * Finds a {@link Pose2d} between all of the parameters.
     * @param poses The translations to analyze
     */
    public Pose2d getMean(Pose2d... poses) {
        if (poses.length == 0) return new Pose2d();
        if (poses.length == 1) return poses[0];

        Pose2d interpolated = poses[0].interpolate(poses[1], 0.5);
        double done = 2;
        double total = 2;
        for (int i = 2; i < poses.length; i++) {
            total++;
            interpolated = poses[i].interpolate(interpolated, done / total);
            done++;
        }

        return interpolated;
    }
}