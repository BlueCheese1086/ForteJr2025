package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        double launchPercent = 0;
        Temperature launchTemperature = Celsius.zero();
        Current launchCurrent = Amps.zero();
        Voltage launchVoltage = Volts.zero();

        double feedPercent = 0;
        Temperature feedTemperature = Celsius.zero();
        Current feedCurrent = Amps.zero();
        Voltage feedVoltage = Volts.zero();
    }

    public void updateInputs();

    public double getLaunchPercent();

    public void setLaunchPercent(double speed);

    public Temperature getLaunchTemperature();

    public Current getLaunchCurrent();

    public Voltage getLaunchVoltage();

    public double getFeedPercent();

    public void setFeedPercent(double speed);

    public Temperature getFeedTemperature();

    public Current getFeedCurrent();

    public Voltage getFeedVoltage();
}