package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class DrivetrainIOSparkMax implements DrivetrainIO {
    private SparkMax flMotor;
    private SparkMax frMotor;
    private SparkMax blMotor;
    private SparkMax brMotor;

    private RelativeEncoder flEncoder;
    private RelativeEncoder frEncoder;

    private DrivetrainIOInputsAutoLogged inputs;

    public DrivetrainIOSparkMax() {
        // Initializing the motors
        flMotor = new SparkMax(DriveConstants.frontLeftID,  MotorType.kBrushless);
        frMotor = new SparkMax(DriveConstants.frontRightID, MotorType.kBrushless);
        blMotor = new SparkMax(DriveConstants.backLeftID,   MotorType.kBrushless);
        brMotor = new SparkMax(DriveConstants.backRightID,  MotorType.kBrushless);

        // Making the config object
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);

        frMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        config.follow(frMotor);
        
        brMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.disableFollowerMode();
        config.inverted(true);

        flMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(flMotor);

        blMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Getting the encoder for each motor
        flEncoder = flMotor.getEncoder();
        frEncoder = frMotor.getEncoder();

        inputs = new DrivetrainIOInputsAutoLogged();
    }

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

        Logger.processInputs("DrivetrainSparkMaxInputs", inputs);
    }

    @Override
    public Current getLeftCurrent() {
        return Amps.of((flMotor.getOutputCurrent() + blMotor.getOutputCurrent()) / 2.0);
    }

    @Override
    public Distance getLeftPosition() {
        return Meters.of(flEncoder.getPosition() * DriveConstants.rotToMeters);
    }

    @Override
    public Temperature getLeftTemperature() {
        return Celsius.of(flMotor.getMotorTemperature());
    }

    @Override
    public LinearVelocity getLeftVelocity() {
        return MetersPerSecond.of(flEncoder.getVelocity() * DriveConstants.rotToMeters / 60);
    }

    @Override
    public Voltage getLeftVoltage() {
        return Volts.of(flMotor.getBusVoltage());
    }

    @Override
    public void setLeftVoltage(double volts) {
        flMotor.setVoltage(volts);
    }

    @Override
    public Current getRightCurrent() {
        return Amps.of((frMotor.getOutputCurrent() + brMotor.getOutputCurrent()) / 2.0);
    }

    @Override
    public Distance getRightPosition() {
        return Meters.of(frEncoder.getPosition() * DriveConstants.rotToMeters);
    }

    @Override
    public Temperature getRightTemperature() {
        return Celsius.of(frMotor.getMotorTemperature());
    }

    @Override
    public LinearVelocity getRightVelocity() {
        return MetersPerSecond.of(frEncoder.getVelocity() * DriveConstants.rotToMeters / 60);
    }

    @Override
    public Voltage getRightVoltage() {
        return Volts.of(frMotor.getBusVoltage());
    }

    @Override
    public void setRightVoltage(double volts) {
        frMotor.setVoltage(volts);
    }
}