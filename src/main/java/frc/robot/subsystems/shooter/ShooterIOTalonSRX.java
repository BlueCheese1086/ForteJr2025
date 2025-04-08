package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonSRX implements ShooterIO {
    private TalonSRX feedMotor;
    private TalonSRX launchMotor;

    private ShooterIOInputs inputs;

    public ShooterIOTalonSRX(int feedId, int launchId) {
        feedMotor = new TalonSRX(feedId);
        launchMotor = new TalonSRX(launchId);

        feedMotor.setInverted(true);
        feedMotor.setNeutralMode(NeutralMode.Brake);

        launchMotor.setInverted(true);
        launchMotor.setNeutralMode(NeutralMode.Brake);

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
        return launchMotor.getMotorOutputPercent();
    }

    @Override
    public void setLaunchPercent(double speed) {
        launchMotor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public Temperature getLaunchTemperature() {
        return Celsius.of(launchMotor.getTemperature());
    }

    @Override
    public Current getLaunchCurrent() {
        return Amps.of(launchMotor.getStatorCurrent());
    }

    @Override
    public Voltage getLaunchVoltage() {
        return Volts.of(launchMotor.getMotorOutputVoltage());
    }

    @Override
    public double getFeedPercent() {
        return feedMotor.getMotorOutputPercent();
    }

    @Override
    public void setFeedPercent(double speed) {
        feedMotor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public Temperature getFeedTemperature() {
        return Celsius.of(feedMotor.getTemperature());
    }

    @Override
    public Current getFeedCurrent() {
        return Amps.of(feedMotor.getStatorCurrent());
    }

    @Override
    public Voltage getFeedVoltage() {
        return Volts.of(feedMotor.getMotorOutputVoltage());
    }
}