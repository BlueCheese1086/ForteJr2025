package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.AdjustableValues;

public class ShooterIOSim implements ShooterIO {
    private DCMotorSim feedMotor;
    private DCMotorSim launchMotor;

    private PIDController feedController;
    private PIDController launchController;

    private boolean feedOpenLoop = true;
    private boolean launchOpenLoop = true;

    public ShooterIOSim() {
        feedMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 1, 1), DCMotor.getNEO(1));
        launchMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 1, 1), DCMotor.getNEO(1));

        feedController = new PIDController(
            AdjustableValues.getNumber("Feed_kP"),
            AdjustableValues.getNumber("Feed_kI"),
            AdjustableValues.getNumber("Feed_kD"));

        launchController = new PIDController(
            AdjustableValues.getNumber("Launch_kP"),
            AdjustableValues.getNumber("Launch_kI"),
            AdjustableValues.getNumber("Launch_kD"));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Updating PID Values
        if (AdjustableValues.hasChanged("Feed_kP")) feedController.setP(AdjustableValues.getNumber("Feed_kP"));
        if (AdjustableValues.hasChanged("Feed_kI")) feedController.setI(AdjustableValues.getNumber("Feed_kI"));
        if (AdjustableValues.hasChanged("Feed_kD")) feedController.setD(AdjustableValues.getNumber("Feed_kD"));

        if (AdjustableValues.hasChanged("Launch_kP")) launchController.setP(AdjustableValues.getNumber("Launch_kP"));
        if (AdjustableValues.hasChanged("Launch_kI")) launchController.setI(AdjustableValues.getNumber("Launch_kI"));
        if (AdjustableValues.hasChanged("Launch_kD")) launchController.setD(AdjustableValues.getNumber("Launch_kD"));

        // Driving the feed motor if it is not in open loop control
        if (!feedOpenLoop) {
            feedMotor.setInputVoltage(feedController.calculate(feedMotor.getAngularVelocityRadPerSec()) + AdjustableValues.getNumber("Feed_kFF"));
        }

        // Driving the launch motor if it is not in open loop control
        if (!launchOpenLoop) {
            launchMotor.setInputVoltage(launchController.calculate(launchMotor.getAngularVelocityRadPerSec()) + AdjustableValues.getNumber("Launch_kFF"));
        }

        // Updating the states of the motors
        feedMotor.update(0.02);
        launchMotor.update(0.02);

        // Updating inputs
        inputs.feedCurrent = Amps.of(feedMotor.getCurrentDrawAmps());
        inputs.feedPercent = feedMotor.getAngularVelocityRPM() / ShooterConstants.maxClosedFeedSpeed.in(RPM);
        inputs.feedPosition = feedMotor.getAngularPosition();
        inputs.feedVelocity = feedMotor.getAngularVelocity();
        inputs.feedVoltage = Volts.of(feedMotor.getInputVoltage());

        inputs.launchCurrent = Amps.of(launchMotor.getCurrentDrawAmps());
        inputs.launchPercent = launchMotor.getAngularVelocityRPM() / ShooterConstants.maxClosedLaunchSpeed.in(RPM);
        inputs.launchPosition = launchMotor.getAngularPosition();
        inputs.launchVelocity = launchMotor.getAngularVelocity();
        inputs.launchVoltage = Volts.of(launchMotor.getInputVoltage());
    }

    @Override
    public void setFeedPercent(double percent) {
        feedOpenLoop = true;

        // Simulates brake mode
        if (percent == 0) feedMotor.setState(feedMotor.getAngularPositionRad(), 0);
        
        feedMotor.setInputVoltage(percent * RobotController.getInputVoltage());
    }

    @Override
    public void setLaunchPercent(double percent) {
        launchOpenLoop = true;

        // Simulates brake mode
        if (percent == 0) launchMotor.setState(launchMotor.getAngularPositionRad(), 0);

        launchMotor.setInputVoltage(percent * RobotController.getInputVoltage());
    }

    @Override
    public void setFeedSpeed(AngularVelocity velocity) {
        feedOpenLoop = false;

        // Simulates brake mode
        if (velocity.magnitude() == 0) feedMotor.setState(feedMotor.getAngularPositionRad(), 0);

        feedController.setSetpoint(velocity.in(RadiansPerSecond));
    }

    @Override
    public void setLaunchSpeed(AngularVelocity velocity) {
        launchOpenLoop = false;

        // Simulates brake mode
        if (velocity.magnitude() == 0) launchMotor.setState(launchMotor.getAngularPositionRad(), 0);

        launchController.setSetpoint(velocity.in(RadiansPerSecond));
    }
}