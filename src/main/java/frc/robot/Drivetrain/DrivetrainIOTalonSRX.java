package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class DrivetrainIOTalonSRX implements DrivetrainIO {
    private TalonSRX flMotor;
    private TalonSRX frMotor;
    private TalonSRX blMotor;
    private TalonSRX brMotor;

    private DrivetrainIOInputsAutoLogged inputs;

    /**
     * TalonSRXs are brushed controllers, and do not have a built-in encoder.
     * This results in robots without external encoders to not know their wheel positions or velocity.
     */
    public DrivetrainIOTalonSRX() {
        // Initializing the motors
        flMotor = new TalonSRX(DriveConstants.frontLeftID);
        frMotor = new TalonSRX(DriveConstants.frontRightID);
        blMotor = new TalonSRX(DriveConstants.backLeftID);
        brMotor = new TalonSRX(DriveConstants.backRightID);

        // Resetting each TalonSRX's settings.
        // The loops make sure that the reset is completed.
        while (flMotor.configFactoryDefault() != ErrorCode.OK) {}
        while (frMotor.configFactoryDefault() != ErrorCode.OK) {}
        while (blMotor.configFactoryDefault() != ErrorCode.OK) {}
        while (brMotor.configFactoryDefault() != ErrorCode.OK) {}

        // Inverting the motors
        flMotor.setInverted(true);
        frMotor.setInverted(false);
        blMotor.setInverted(true);
        brMotor.setInverted(false);

        // Putting the motors into brake mode.
        flMotor.setNeutralMode(NeutralMode.Brake);
        frMotor.setNeutralMode(NeutralMode.Brake);
        blMotor.setNeutralMode(NeutralMode.Brake);
        brMotor.setNeutralMode(NeutralMode.Brake);

        // Having the back motors follow the front motors.
        blMotor.follow(flMotor);
        brMotor.follow(frMotor);

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

        Logger.processInputs("DrivetrainTalonSRXInputs", inputs);
    }

    @Override
    public Current getLeftCurrent() {
        return Amps.of((flMotor.getStatorCurrent() + blMotor.getStatorCurrent()) / 2.0);
    }

    @Override
    public Distance getLeftPosition() {
        return Meters.zero();
    }

    @Override
    public Temperature getLeftTemperature() {
        return Units.Celsius.of(flMotor.getTemperature());
    }

    @Override
    public LinearVelocity getLeftVelocity() {
        return MetersPerSecond.zero();
    }

    @Override
    public Voltage getLeftVoltage() {
        return Volts.of(flMotor.getMotorOutputVoltage());
    }

    @Override
    public void setLeftVoltage(double volts) {
        flMotor.set(ControlMode.PercentOutput, volts / flMotor.getBusVoltage());
    }

    @Override
    public Current getRightCurrent() {
        return Amps.of((frMotor.getStatorCurrent() + brMotor.getStatorCurrent()) / 2.0);
    }

    @Override
    public Distance getRightPosition() {
        return Meters.zero();
    }

    @Override
    public Temperature getRightTemperature() {
        return Units.Celsius.of(frMotor.getTemperature());
    }

    @Override
    public LinearVelocity getRightVelocity() {
        return MetersPerSecond.zero();
    }

    @Override
    public Voltage getRightVoltage() {
        return Volts.of(frMotor.getMotorOutputVoltage());
    }

    @Override
    public void setRightVoltage(double volts) {
        frMotor.set(ControlMode.PercentOutput, volts / frMotor.getBusVoltage());
    }
}