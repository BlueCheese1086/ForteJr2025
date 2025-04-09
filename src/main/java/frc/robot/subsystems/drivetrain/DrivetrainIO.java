package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface DrivetrainIO {
    @AutoLog
    public class DrivetrainIOInputs {
        Current leftCurrent = Amps.zero();
        Distance leftPosition = Meters.zero();
        Temperature leftTemperature = Celsius.zero();
        LinearVelocity leftVelocity = MetersPerSecond.zero();
        Voltage leftVoltage = Volts.zero();

        Current rightCurrent = Amps.zero();
        Distance rightPosition = Meters.zero();
        Temperature rightTemperature = Celsius.zero();
        LinearVelocity rightVelocity = MetersPerSecond.zero();
        Voltage rightVoltage = Volts.zero();
    }

    /** 
     * Updates the logged inputs for the drivetrian.
     * 
     * @param inputs The inputs object to update.
     */
    public void updateInputs(DrivetrainIOInputs inputs);

    /** Sets the voltage output of the left side of the robot. */
    public void setLeftVoltage(Voltage volts);

    /** Sets the voltage output of the right side of the robot. */
    public void setRightVoltage(Voltage volts);

    /** Sets the velocity of the left side of the robot. */
    public void setLeftSpeed(LinearVelocity velocity);

    /** Sets the velocity of the right side of the robot. */
    public void setRightSpeed(LinearVelocity velocity);
}