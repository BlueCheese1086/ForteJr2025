package frc.robot.subsystems.drivetrain;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface DrivetrainIO {
    @AutoLog
    public static class DrivetrainIOInputs {
        public Current leftCurrent;
        public Distance leftPosition;
        public Temperature leftTemperature;
        public LinearVelocity leftVelocity;
        public Voltage leftVoltage;

        public Current rightCurrent;
        public Distance rightPosition;
        public Temperature rightTemperature;
        public LinearVelocity rightVelocity;
        public Voltage rightVoltage;
    }

    /** 
     * Updates the logged inputs for the drivetrian.
     * 
     * @param inputs The inputs object to update.
     */
    public void updateInputs();

    /**
     * Returns the current draw of the left side of the robot.
     * 
     * @return The current draw of the left side of the robot in Amps.
     */
    public Current getLeftCurrent();

    /**
     * Returns the distance that the left side of the robot has traveled.
     * 
     * @return The distance that the left side of the robot has traveled in meters.
     */
    public Distance getLeftPosition();
    
    public Temperature getLeftTemperature();
    
    public LinearVelocity getLeftVelocity();

    public Voltage getLeftVoltage();
    public void setLeftVoltage(double volts);

    public Current getRightCurrent();
    
    public Distance getRightPosition();
    
    public Temperature getRightTemperature();

    public LinearVelocity getRightVelocity();
    
    public Voltage getRightVoltage();
    public void setRightVoltage(double volts);
}