package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
    private TalonFX feedMotor;
    private TalonFX launchMotor;

    private ShooterIOInputs inputs;

    public ShooterIOTalonFX(int feedId, int launchId) {
        feedMotor = new TalonFX(feedId);
        launchMotor = new TalonFX(launchId);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        feedMotor.getConfigurator().apply(config);
        launchMotor.getConfigurator().apply(config);

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
        return launchMotor.get();
    }

    @Override
    public void setLaunchPercent(double speed) {
        launchMotor.set(speed);
    }

    @Override
    public Temperature getLaunchTemperature() {
        return launchMotor.getDeviceTemp().getValue();
    }

    @Override
    public Current getLaunchCurrent() {
        return launchMotor.getStatorCurrent().getValue();
    }

    @Override
    public Voltage getLaunchVoltage() {
        return launchMotor.getMotorVoltage().getValue();
    }

    @Override
    public double getFeedPercent() {
        return feedMotor.get();
    }

    @Override
    public void setFeedPercent(double speed) {
        feedMotor.set(speed);
    }

    @Override
    public Temperature getFeedTemperature() {
        return feedMotor.getDeviceTemp().getValue();
    }

    @Override
    public Current getFeedCurrent() {
        return feedMotor.getStatorCurrent().getValue();
    }

    @Override
    public Voltage getFeedVoltage() {
        return feedMotor.getMotorVoltage().getValue();
    }
}