package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalonSRX implements ShooterIO {
    private TalonSRX feed;
    private TalonSRX launch;

    // Alert for when the user uses closed loop control on the brushed motor controllers.
    private Alert noClosedLoop = new Alert("TalonSRX unable to use closed loop control.  Rolling back to percent output.", Alert.AlertType.kWarning);

    public ShooterIOTalonSRX(int feedId, int launchId) {
        feed = new TalonSRX(feedId);
        launch = new TalonSRX(launchId);

        feed.setInverted(true);
        feed.setNeutralMode(NeutralMode.Brake);

        launch.setInverted(true);
        launch.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Updating inputs
        inputs.feedCurrent = Amps.of(feed.getStatorCurrent());
        inputs.feedPercent = feed.getMotorOutputPercent();
        inputs.feedTemperature = Celsius.of(feed.getTemperature());
        inputs.feedVoltage = Volts.of(feed.getMotorOutputVoltage());

        inputs.launchCurrent = Amps.of(launch.getStatorCurrent());
        inputs.launchPercent = launch.getMotorOutputPercent();
        inputs.launchTemperature = Celsius.of(launch.getTemperature());
        inputs.launchVoltage = Volts.of(launch.getMotorOutputVoltage());
    }

    @Override
    public void setLaunchPercent(double speed) {
        noClosedLoop.set(false);

        launch.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void setFeedPercent(double speed) {
        noClosedLoop.set(false);

        feed.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void setFeedSpeed(AngularVelocity speed) {
        noClosedLoop.set(true);

        feed.set(ControlMode.PercentOutput, speed.div(ShooterConstants.maxFeedSpeed).magnitude());
    }

    @Override
    public void setLaunchSpeed(AngularVelocity speed) {
        noClosedLoop.set(true);

        launch.set(ControlMode.PercentOutput, speed.div(ShooterConstants.maxLaunchSpeed).magnitude());
    }
}