package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
    private DCMotorSim feedMotor;
    private DCMotorSim launchMotor;

    private ShooterIOInputs inputs;

    public ShooterIOSim() {
        feedMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 1, 1), DCMotor.getNEO(1));
        launchMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 1, 1), DCMotor.getNEO(1));

        inputs = new ShooterIOInputs();
    }

    @Override
    public void updateInputs() {
        feedMotor.update(0.02);
        launchMotor.update(0.02);

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
        return launchMotor.getInputVoltage() / RobotController.getInputVoltage();
    }

    @Override
    public void setLaunchPercent(double speed) {
        if (speed == 0) launchMotor.setState(launchMotor.getAngularPositionRad(), 0);

        launchMotor.setInputVoltage(-speed * RobotController.getInputVoltage());
    }

    @Override
    public Temperature getLaunchTemperature() {
        return Celsius.zero();
    }

    @Override
    public Current getLaunchCurrent() {
        return Amps.of(launchMotor.getCurrentDrawAmps());
    }

    @Override
    public Voltage getLaunchVoltage() {
        return Volts.of(launchMotor.getInputVoltage());
    }

    @Override
    public double getFeedPercent() {
        return feedMotor.getInputVoltage() / RobotController.getInputVoltage();
    }

    @Override
    public void setFeedPercent(double speed) {
        if (speed == 0) feedMotor.setState(feedMotor.getAngularPositionRad(), 0);
        
        feedMotor.setInputVoltage(-speed * RobotController.getInputVoltage());
    }

    @Override
    public Temperature getFeedTemperature() {
        return Celsius.zero();
    }

    @Override
    public Current getFeedCurrent() {
        return Amps.of(feedMotor.getCurrentDrawAmps());
    }

    @Override
    public Voltage getFeedVoltage() {
        return Volts.of(feedMotor.getInputVoltage());
    }
}