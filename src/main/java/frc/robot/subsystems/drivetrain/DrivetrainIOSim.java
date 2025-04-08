package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import frc.robot.Drivetrain.DrivetrainIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class DrivetrainIOSim implements DrivetrainIO {
    private DifferentialDrivetrainSim driveSim;

    private DrivetrainIOInputsAutoLogged inputs;

    private double leftVolts;
    private double rightVolts;
    
    public DrivetrainIOSim() {
        inputs = new DrivetrainIOInputsAutoLogged();
// 0.366568182
        driveSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDoubleNEOPerSide, KitbotGearing.k8p45, KitbotWheelSize.kEightInch, null);
    }

    /**
     * This function updates the logged inputs.
     * It should be placed in a periodic function somewhere.
     * 
     * @param inputs An instance of the class that contains the inputs that need to be logged.
     */
    @Override
    public void updateInputs() {
        driveSim.setInputs(leftVolts, rightVolts);

        driveSim.update(0.02);

        // Voltages
        inputs.leftVoltage = getLeftVoltage();
        inputs.rightVoltage = getRightVoltage();

        // Positions
        inputs.leftPosition = getLeftPosition();
        inputs.rightPosition = getRightPosition();

        // Current
        inputs.leftCurrent = getLeftCurrent();
        inputs.rightCurrent = getRightCurrent();

        // Temperature
        inputs.leftTemperature = getLeftTemperature();
        inputs.rightTemperature = getRightTemperature();

        // Velocity
        inputs.leftVelocity = getLeftVelocity();
        inputs.rightVelocity = getRightVelocity();

        Logger.processInputs("DrivetrainSimInputs", inputs);
    }

    @Override
    public Current getLeftCurrent() {
        return Amps.of(driveSim.getLeftCurrentDrawAmps());
    }

    @Override
    public Distance getLeftPosition() {
        return Meters.of(driveSim.getLeftPositionMeters());
    }

    @Override
    public Temperature getLeftTemperature() {
        return Celsius.zero();
    }

    @Override
    public Voltage getLeftVoltage() {
        return Volts.of(leftVolts);
    }

    @Override
    public void setLeftVoltage(double volts) {
        leftVolts = volts;
    }

    @Override
    public LinearVelocity getLeftVelocity() {
        return MetersPerSecond.of(driveSim.getLeftVelocityMetersPerSecond());
    }

    @Override
    public Current getRightCurrent() {
        return Amps.of(driveSim.getRightCurrentDrawAmps());
    }

    @Override
    public Distance getRightPosition() {
        return Meters.of(driveSim.getRightPositionMeters());
    }

    @Override
    public Temperature getRightTemperature() {
        return Celsius.zero();
    }

    @Override
    public Voltage getRightVoltage() {
        return Volts.of(rightVolts);
    }

    @Override
    public void setRightVoltage(double volts) {
        rightVolts = volts;
    }

    @Override
    public LinearVelocity getRightVelocity() {
        return MetersPerSecond.of(driveSim.getRightVelocityMetersPerSecond());
    }
}