package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.DrivetrainIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class DrivetrainIOTalonFX implements DrivetrainIO {
    private TalonFX flMotor;
    private TalonFX frMotor;
    private TalonFX blMotor;
    private TalonFX brMotor;

    private DrivetrainIOInputsAutoLogged inputs;

    public DrivetrainIOTalonFX(int frontLeftID, int frontRightID, int backLeftID, int backRightID) {
        // Initializing the motors
        flMotor = new TalonFX(frontLeftID);
        frMotor = new TalonFX(frontRightID);
        blMotor = new TalonFX(backLeftID);
        brMotor = new TalonFX(backRightID);

        // Resetting the motors
        // The loops either force the configuration to finish, or prevent the program from continuing.
        while (flMotor.getConfigurator().apply(new TalonFXConfiguration()) != StatusCode.OK) {}
        while (frMotor.getConfigurator().apply(new TalonFXConfiguration()) != StatusCode.OK) {}
        while (blMotor.getConfigurator().apply(new TalonFXConfiguration()) != StatusCode.OK) {}
        while (brMotor.getConfigurator().apply(new TalonFXConfiguration()) != StatusCode.OK) {}

        // Applying configs to each motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flMotor.getConfigurator().apply(config);
        blMotor.getConfigurator().apply(config);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        frMotor.getConfigurator().apply(config);
        brMotor.getConfigurator().apply(config);

        // Making the back motors follow the front motors.
        blMotor.setControl(new Follower(flMotor.getDeviceID(), false));
        brMotor.setControl(new Follower(frMotor.getDeviceID(), false));

        inputs = new DrivetrainIOInputsAutoLogged();
    }

    /**
     * This function updates the logged inputs.
     * It should be placed in a periodic function somewhere.
     * 
     * @param inputs An instance of the class that contains the inputs that need to be logged.
     */
    @Override
    public void updateInputs() {
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

        Logger.processInputs("DrivetrainTalonFXInputs", inputs);
    }

    @Override
    public Current getLeftCurrent() {
        return flMotor.getStatorCurrent().getValue().plus(blMotor.getStatorCurrent().getValue()).div(2);
    }

    @Override
    public Distance getLeftPosition() {
        return Meters.of(flMotor.getPosition().getValue().in(Rotations) * DriveConstants.rotToMeters);
    }

    @Override
    public Temperature getLeftTemperature() {
        return flMotor.getDeviceTemp().getValue();
    }

    @Override
    public LinearVelocity getLeftVelocity() {
        return MetersPerSecond.of(flMotor.getVelocity().getValue().in(RotationsPerSecond) * DriveConstants.rotToMeters);
    }

    @Override
    public Voltage getLeftVoltage() {
        return flMotor.getMotorVoltage().getValue();
    }

    @Override
    public void setLeftVoltage(double volts) {
        flMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public Current getRightCurrent() {
        return frMotor.getStatorCurrent().getValue().plus(brMotor.getStatorCurrent().getValue()).div(2);
    }

    @Override
    public Distance getRightPosition() {
        return Meters.of(frMotor.getPosition().getValue().in(Rotations) * DriveConstants.rotToMeters);
    }

    @Override
    public Temperature getRightTemperature() {
        return frMotor.getDeviceTemp().getValue();
    }

    @Override
    public LinearVelocity getRightVelocity() {
        return MetersPerSecond.of(frMotor.getVelocity().getValue().in(RotationsPerSecond) * DriveConstants.rotToMeters);
    }

    @Override
    public Voltage getRightVoltage() {
        return frMotor.getMotorVoltage().getValue();
    }

    @Override
    public void setRightVoltage(double volts) {
        frMotor.setControl(new VoltageOut(volts));
    }
}