package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOSparkMax implements ShooterIO {
    private SparkMax feedMotor;
    private SparkMax launchMotor;

    private ShooterIOInputs inputs;

    public ShooterIOSparkMax(int feedId, int launchId) {
        feedMotor = new SparkMax(feedId, MotorType.kBrushless);
        launchMotor = new SparkMax(launchId, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);

        feedMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        launchMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        inputs = new ShooterIOInputs();
    }

    @Override
    public void updateInputs() {
        inputs.feedCurrent = getFeedCurrent();
        inputs.feedPercent = getFeedPercent();
        inputs.feedTemperature = getFeedTemperature();
        inputs.feedVoltage = getFeedVoltage();

        inputs.launchCurrent = getLaunchCurrent();
        inputs.launchPercent = getLaunchPercent();
        inputs.launchTemperature = getLaunchTemperature();
        inputs.launchVoltage = getLaunchVoltage();
    }

    @Override
    public double getLaunchPercent() {
        return launchMotor.getAppliedOutput();
    }

    @Override
    public void setLaunchPercent(double speed) {
        launchMotor.set(speed);
    }

    @Override
    public Temperature getLaunchTemperature() {
        return Celsius.of(launchMotor.getMotorTemperature());
    }

    @Override
    public Current getLaunchCurrent() {
        return Amps.of(launchMotor.getOutputCurrent());
    }

    @Override
    public Voltage getLaunchVoltage() {
        return Volts.of(launchMotor.getBusVoltage() * launchMotor.getAppliedOutput());
    }

    @Override
    public double getFeedPercent() {
        return feedMotor.getAppliedOutput();
    }

    @Override
    public void setFeedPercent(double speed) {
        feedMotor.set(speed);
    }

    @Override
    public Temperature getFeedTemperature() {
        return Celsius.of(feedMotor.getMotorTemperature());
    }

    @Override
    public Current getFeedCurrent() {
        return Amps.of(feedMotor.getOutputCurrent());
    }

    @Override
    public Voltage getFeedVoltage() {
        return Volts.of(feedMotor.getBusVoltage() * feedMotor.getAppliedOutput());
    }
}