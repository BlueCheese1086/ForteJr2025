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
        Current flCurrent = Amps.zero();
        Current frCurrent = Amps.zero();
        Current blCurrent = Amps.zero();
        Current brCurrent = Amps.zero();

        Distance flPosition = Meters.zero();
        Distance frPosition = Meters.zero();
        Distance blPosition = Meters.zero();
        Distance brPosition = Meters.zero();

        Temperature flTemperature = Celsius.zero();
        Temperature frTemperature = Celsius.zero();
        Temperature blTemperature = Celsius.zero();
        Temperature brTemperature = Celsius.zero();

        LinearVelocity flVelocity = MetersPerSecond.zero();
        LinearVelocity frVelocity = MetersPerSecond.zero();
        LinearVelocity blVelocity = MetersPerSecond.zero();
        LinearVelocity brVelocity = MetersPerSecond.zero();

        Voltage flVoltage = Volts.zero();
        Voltage frVoltage = Volts.zero();
        Voltage blVoltage = Volts.zero();
        Voltage brVoltage = Volts.zero();
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