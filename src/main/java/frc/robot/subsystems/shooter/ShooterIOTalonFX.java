package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.AdjustableValues;

public class ShooterIOTalonFX implements ShooterIO {
    private TalonFX feed;
    private TalonFX launch;

    // Control methods
    private DutyCycleOut feedOpenLoop = new DutyCycleOut(0);
    private DutyCycleOut launchOpenLoop = new DutyCycleOut(0);
    private VelocityVoltage feedClosedLoop = new VelocityVoltage(0).withSlot(0);
    private VelocityVoltage launchClosedLoop = new VelocityVoltage(0).withSlot(0);

    public ShooterIOTalonFX(int feedId, int launchId) {
        feed = new TalonFX(feedId);
        launch = new TalonFX(launchId);

        TalonFXConfiguration feedConfig = new TalonFXConfiguration();

        feedConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feedConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        feedConfig.Slot0.kP = AdjustableValues.getNumber("Feed_kP");
        feedConfig.Slot0.kI = ShooterConstants.feedI;
        feedConfig.Slot0.kD = ShooterConstants.feedD;

        TalonFXConfiguration launchConfig = new TalonFXConfiguration();

        launchConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        launchConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        launchConfig.Slot0.kP = ShooterConstants.feedP;
        launchConfig.Slot0.kI = ShooterConstants.feedI;
        launchConfig.Slot0.kD = ShooterConstants.feedD;

        feed.getConfigurator().apply(feedConfig);
        launch.getConfigurator().apply(launchConfig);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Updating PID values
        Slot0Configs config = new Slot0Configs();
        if (AdjustableValues.hasChanged("Feed_kP")) config.kP = AdjustableValues.getNumber("Feed_kP");
        if (AdjustableValues.hasChanged("Feed_kI")) config.kI = AdjustableValues.getNumber("Feed_kI");
        if (AdjustableValues.hasChanged("Feed_kD")) config.kD = AdjustableValues.getNumber("Feed_kD");
        if (!config.equals(new Slot0Configs())) feed.getConfigurator().apply(config);

        config = new Slot0Configs();
        if (AdjustableValues.hasChanged("Launch_kP")) config.kP = AdjustableValues.getNumber("Launch_kP");
        if (AdjustableValues.hasChanged("Launch_kI")) config.kI = AdjustableValues.getNumber("Launch_kI");
        if (AdjustableValues.hasChanged("Launch_kD")) config.kD = AdjustableValues.getNumber("Launch_kD");
        if (!config.equals(new Slot0Configs())) launch.getConfigurator().apply(config);

        if (AdjustableValues.hasChanged("Feed_kFF")) feedClosedLoop.withFeedForward(AdjustableValues.getNumber("Feed_kFF"));
        if (AdjustableValues.hasChanged("Launch_kFF")) launchClosedLoop.withFeedForward(AdjustableValues.getNumber("Launch_kFF"));

        // Updating inputs
        inputs.feedCurrent = feed.getStatorCurrent().getValue();
        inputs.feedPercent = feed.get();
        inputs.feedPosition = feed.getPosition().getValue();
        inputs.feedTemperature = feed.getDeviceTemp().getValue();
        inputs.feedVelocity = feed.getVelocity().getValue();
        inputs.feedVoltage = feed.getMotorVoltage().getValue();

        inputs.launchCurrent = launch.getStatorCurrent().getValue();
        inputs.launchPercent = launch.get();
        inputs.launchPosition = launch.getPosition().getValue();
        inputs.launchTemperature = launch.getDeviceTemp().getValue();
        inputs.launchVelocity = launch.getVelocity().getValue();
        inputs.launchVoltage = launch.getMotorVoltage().getValue();
    }

    @Override
    public void setFeedPercent(double speed) {
        feed.setControl(feedOpenLoop.withOutput(speed));
    }

    @Override
    public void setLaunchPercent(double speed) {
        launch.setControl(launchOpenLoop.withOutput(speed));
    }

    @Override
    public void setFeedSpeed(AngularVelocity speed) {
        feed.setControl(feedClosedLoop.withVelocity(speed));
    }

    @Override
    public void setLaunchSpeed(AngularVelocity speed) {
        launch.setControl(launchClosedLoop.withVelocity(speed));
    }
}