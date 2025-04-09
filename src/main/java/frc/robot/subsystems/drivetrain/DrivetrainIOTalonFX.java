package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.AdjustableValues;

public class DrivetrainIOTalonFX implements DrivetrainIO {
    private TalonFX flMotor;
    private TalonFX frMotor;
    private TalonFX blMotor;
    private TalonFX brMotor;

    // Control methods
    private VoltageOut leftOpenLoop = new VoltageOut(0);
    private VoltageOut rightOpenLoop = new VoltageOut(0);
    private VelocityVoltage leftClosedLoop = new VelocityVoltage(0).withSlot(0);
    private VelocityVoltage rightClosedLoop = new VelocityVoltage(0).withSlot(0);

    public DrivetrainIOTalonFX(int frontLeftID, int frontRightID, int backLeftID, int backRightID) {
        // Initializing the motors
        flMotor = new TalonFX(frontLeftID);
        frMotor = new TalonFX(frontRightID);
        blMotor = new TalonFX(backLeftID);
        brMotor = new TalonFX(backRightID);

        // Applying configs to each motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = DriveConstants.gearRatio;
        config.Slot0.kP = AdjustableValues.getNumber("Drive_LeftkP");
        config.Slot0.kI = AdjustableValues.getNumber("Drive_LeftkI");
        config.Slot0.kD = AdjustableValues.getNumber("Drive_LeftkD");

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        flMotor.getConfigurator().apply(config);
        blMotor.getConfigurator().apply(config);

        config.Slot0.kP = AdjustableValues.getNumber("Drive_RightkP");
        config.Slot0.kI = AdjustableValues.getNumber("Drive_RightkI");
        config.Slot0.kD = AdjustableValues.getNumber("Drive_RightkD");
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        frMotor.getConfigurator().apply(config);
        brMotor.getConfigurator().apply(config);

        // Making the back motors follow the front motors.
        blMotor.setControl(new Follower(flMotor.getDeviceID(), false));
        brMotor.setControl(new Follower(frMotor.getDeviceID(), false));

        leftClosedLoop.withFeedForward(AdjustableValues.getNumber("Drive_LeftkFF"));
        rightClosedLoop.withFeedForward(AdjustableValues.getNumber("Drive_RightkFF"));
    }

    /**
     * This function updates the logged inputs.
     * It should be placed in a periodic function somewhere.
     * 
     * @param inputs An instance of the class that contains the inputs that need to be logged.
     */
    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        // Updating PID values
        Slot0Configs config = new Slot0Configs();
        if (AdjustableValues.hasChanged("Drive_LeftkP")) config.kP = AdjustableValues.getNumber("Drive_LeftkP");
        if (AdjustableValues.hasChanged("Drive_LeftkI")) config.kI = AdjustableValues.getNumber("Drive_LeftkI");
        if (AdjustableValues.hasChanged("Drive_LeftkD")) config.kD = AdjustableValues.getNumber("Drive_LeftkD");
        if (!config.equals(new Slot0Configs())) flMotor.getConfigurator().apply(config);

        config = new Slot0Configs();
        if (AdjustableValues.hasChanged("Drive_RightkP")) config.kP = AdjustableValues.getNumber("Drive_RightkP");
        if (AdjustableValues.hasChanged("Drive_RightkI")) config.kI = AdjustableValues.getNumber("Drive_RightkI");
        if (AdjustableValues.hasChanged("Drive_RightkD")) config.kD = AdjustableValues.getNumber("Drive_RightkD");
        if (!config.equals(new Slot0Configs())) frMotor.getConfigurator().apply(config);

        if (AdjustableValues.hasChanged("Drive_LeftkFF")) leftClosedLoop.withFeedForward(AdjustableValues.getNumber("Drive_LeftkFF"));
        if (AdjustableValues.hasChanged("Drive_RightkFF")) rightClosedLoop.withFeedForward(AdjustableValues.getNumber("Drive_RightkFF"));

        // Updating inputs
        // Voltages
        inputs.leftVoltage = flMotor.getMotorVoltage().getValue();
        inputs.rightVoltage = frMotor.getMotorVoltage().getValue();

        // Positions
        inputs.leftPosition = DriveConstants.wheelRadius.times(flMotor.getPosition().getValue().in(Radians));
        inputs.rightPosition = DriveConstants.wheelRadius.times(frMotor.getPosition().getValue().in(Radians));

        // Current
        inputs.leftCurrent = flMotor.getStatorCurrent().getValue();
        inputs.rightCurrent = frMotor.getStatorCurrent().getValue();

        // Temperature
        inputs.leftTemperature = flMotor.getDeviceTemp().getValue();
        inputs.rightTemperature = frMotor.getDeviceTemp().getValue();

        // Velocity
        inputs.leftVelocity = MetersPerSecond.of(flMotor.getVelocity().getValue().in(RadiansPerSecond) * DriveConstants.wheelRadius.in(Meters));
        inputs.rightVelocity = MetersPerSecond.of(frMotor.getVelocity().getValue().in(RadiansPerSecond) * DriveConstants.wheelRadius.in(Meters));
    }

    @Override
    public void setLeftVoltage(Voltage volts) {
        flMotor.setControl(leftOpenLoop.withOutput(volts));
    }

    @Override
    public void setRightVoltage(Voltage volts) {
        frMotor.setControl(rightOpenLoop.withOutput(volts));
    }

    @Override
    public void setLeftSpeed(LinearVelocity velocity) {
        flMotor.setControl(leftClosedLoop.withVelocity(velocity.div(DriveConstants.wheelRadius).magnitude()));
    }

    @Override
    public void setRightSpeed(LinearVelocity velocity) {
        frMotor.setControl(rightClosedLoop.withVelocity(velocity.div(DriveConstants.wheelRadius).magnitude()));
    }
}