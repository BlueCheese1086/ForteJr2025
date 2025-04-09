package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        Current feedCurrent = Amps.zero();
        double feedPercent = 0;
        Angle feedPosition = Radians.zero();
        Temperature feedTemperature = Celsius.zero();
        AngularVelocity feedVelocity = RPM.zero();
        Voltage feedVoltage = Volts.zero();

        Current launchCurrent = Amps.zero();
        double launchPercent = 0;
        Angle launchPosition = Radians.zero();
        Temperature launchTemperature = Celsius.zero();
        AngularVelocity launchVelocity = RPM.zero();
        Voltage launchVoltage = Volts.zero();
    }

    /** Updates a set of {@link ShooterIOInputs} with up-to-date values. */
    public void updateInputs(ShooterIOInputs inputs);

    /** Sets the open loop percent output of the feed motor. */
    public void setFeedPercent(double percent);

    /** Sets the open loop percent output of the launch motor. */
    public void setLaunchPercent(double percent);

    /** Sets the closed loop velocity output of the feed motor. */
    public void setFeedSpeed(AngularVelocity speed);

    /** Sets the closed loop velocity output of the launch motor. */
    public void setLaunchSpeed(AngularVelocity speed);
}