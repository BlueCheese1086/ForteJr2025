package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.AdjustableValues;

public class ShooterIOSparkMax implements ShooterIO {
    private SparkMax feedMotor;
    private SparkMax launchMotor;

    private RelativeEncoder feedEncoder;
    private RelativeEncoder launchEncoder;

    private SparkClosedLoopController feedController;
    private SparkClosedLoopController launchController;

    public ShooterIOSparkMax(int feedId, int launchId) {
        feedMotor = new SparkMax(feedId, MotorType.kBrushless);
        launchMotor = new SparkMax(launchId, MotorType.kBrushless);

        SparkMaxConfig feedConfig = new SparkMaxConfig();
        feedConfig.idleMode(IdleMode.kBrake);
        feedConfig.inverted(true);
        feedConfig.closedLoop.p(ShooterConstants.feedP);
        feedConfig.closedLoop.i(ShooterConstants.feedI);
        feedConfig.closedLoop.d(ShooterConstants.feedD);
        feedConfig.closedLoop.velocityFF(ShooterConstants.feedFF);
        feedMotor.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig launchConfig = new SparkMaxConfig();
        launchConfig.idleMode(IdleMode.kBrake);
        launchConfig.inverted(true);
        launchConfig.closedLoop.p(ShooterConstants.launchP);
        launchConfig.closedLoop.i(ShooterConstants.launchI);
        launchConfig.closedLoop.d(ShooterConstants.launchD);
        launchConfig.closedLoop.velocityFF(ShooterConstants.launchFF);
        launchMotor.configure(launchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feedEncoder = feedMotor.getEncoder();
        launchEncoder = launchMotor.getEncoder();

        feedController = feedMotor.getClosedLoopController();
        launchController = launchMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Updating PID values
        SparkMaxConfig config = new SparkMaxConfig();
        if (AdjustableValues.hasChanged("Feed_kP")) config.closedLoop.p(AdjustableValues.getNumber("Feed_kP"));
        if (AdjustableValues.hasChanged("Feed_kI")) config.closedLoop.i(AdjustableValues.getNumber("Feed_kI"));
        if (AdjustableValues.hasChanged("Feed_kD")) config.closedLoop.d(AdjustableValues.getNumber("Feed_kD"));
        if (AdjustableValues.hasChanged("Feed_kFF")) config.closedLoop.velocityFF(AdjustableValues.getNumber("Feed_kFF"));
        if (!config.equals(new SparkMaxConfig())) feedMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config = new SparkMaxConfig();
        if (AdjustableValues.hasChanged("Launch_kP")) config.closedLoop.p(AdjustableValues.getNumber("Launch_kP"));
        if (AdjustableValues.hasChanged("Launch_kI")) config.closedLoop.i(AdjustableValues.getNumber("Launch_kI"));
        if (AdjustableValues.hasChanged("Launch_kD")) config.closedLoop.d(AdjustableValues.getNumber("Launch_kD"));
        if (AdjustableValues.hasChanged("Launch_kFF")) config.closedLoop.velocityFF(AdjustableValues.getNumber("Launch_kFF"));
        if (!config.equals(new SparkMaxConfig())) launchMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Updating inputs
        inputs.feedCurrent = Amps.of(feedMotor.getOutputCurrent());
        inputs.feedPercent = feedMotor.getAppliedOutput();
        inputs.feedPosition = Radians.of(feedEncoder.getPosition());
        inputs.feedTemperature = Celsius.of(feedMotor.getMotorTemperature());
        inputs.feedVelocity = RadiansPerSecond.of(feedEncoder.getVelocity());
        inputs.feedVoltage = Volts.of(inputs.feedPercent * feedMotor.getBusVoltage());

        inputs.launchCurrent = Amps.of(launchMotor.getOutputCurrent());
        inputs.launchPercent = launchMotor.getAppliedOutput();
        inputs.launchPosition = Radians.of(launchEncoder.getPosition());
        inputs.launchTemperature = Celsius.of(launchMotor.getMotorTemperature());
        inputs.launchVelocity = RadiansPerSecond.of(launchEncoder.getVelocity());
        inputs.launchVoltage = Volts.of(inputs.launchPercent * launchMotor.getBusVoltage());
    }

    @Override
    public void setFeedPercent(double percent) {
        feedMotor.set(percent);
    }

    @Override
    public void setLaunchPercent(double percent) {
        launchMotor.set(percent);
    }

    @Override
    public void setFeedSpeed(AngularVelocity speed) {
        feedController.setReference(speed.in(RadiansPerSecond), ControlType.kVelocity);
    }

    @Override
    public void setLaunchSpeed(AngularVelocity speed) {
        launchController.setReference(speed.in(RadiansPerSecond), ControlType.kVelocity);
    }
}