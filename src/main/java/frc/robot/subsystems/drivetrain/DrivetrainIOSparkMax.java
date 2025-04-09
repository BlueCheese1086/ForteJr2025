package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.AdjustableValues;

public class DrivetrainIOSparkMax implements DrivetrainIO {
    private SparkMax flMotor;
    private SparkMax frMotor;
    private SparkMax blMotor;
    private SparkMax brMotor;

    private RelativeEncoder flEncoder;
    private RelativeEncoder frEncoder;

    private SparkClosedLoopController flController;
    private SparkClosedLoopController frController;

    public DrivetrainIOSparkMax(int frontLeftID, int frontRightID, int backLeftID, int backRightID) {
        // Initializing the motors
        flMotor = new SparkMax(frontLeftID,  MotorType.kBrushless);
        frMotor = new SparkMax(frontRightID, MotorType.kBrushless);
        blMotor = new SparkMax(backLeftID,   MotorType.kBrushless);
        brMotor = new SparkMax(backRightID,  MotorType.kBrushless);

        // Making the config object
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);
        config.closedLoop.p(AdjustableValues.getNumber("Drive_kP"));
        config.closedLoop.i(AdjustableValues.getNumber("Drive_kI"));
        config.closedLoop.d(AdjustableValues.getNumber("Drive_kD"));
        config.closedLoop.velocityFF(AdjustableValues.getNumber("Drive_kFF"));

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

        // Getting the controllers for each motor
        flController = flMotor.getClosedLoopController();
        frController = frMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        // Updating PID values
        SparkMaxConfig config = new SparkMaxConfig();
        if (AdjustableValues.hasChanged("Drive_LeftkP")) config.closedLoop.p(AdjustableValues.getNumber("Drive_LeftkP"));
        if (AdjustableValues.hasChanged("Drive_LeftkI")) config.closedLoop.i(AdjustableValues.getNumber("Drive_LeftkI"));
        if (AdjustableValues.hasChanged("Drive_LeftkD")) config.closedLoop.d(AdjustableValues.getNumber("Drive_LeftkD"));
        if (AdjustableValues.hasChanged("Drive_LeftkFF")) config.closedLoop.velocityFF(AdjustableValues.getNumber("Drive_LeftkFF"));
        if (!config.equals(new SparkMaxConfig())) flMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config = new SparkMaxConfig();
        if (AdjustableValues.hasChanged("Drive_RightkP")) config.closedLoop.p(AdjustableValues.getNumber("Drive_RightkP"));
        if (AdjustableValues.hasChanged("Drive_RightkI")) config.closedLoop.i(AdjustableValues.getNumber("Drive_RightkI"));
        if (AdjustableValues.hasChanged("Drive_RightkD")) config.closedLoop.d(AdjustableValues.getNumber("Drive_RightkD"));
        if (AdjustableValues.hasChanged("Drive_RightkFF")) config.closedLoop.velocityFF(AdjustableValues.getNumber("Drive_RightkFF"));
        if (!config.equals(new SparkMaxConfig())) frMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Updating inputs
        // Voltages
        inputs.leftVoltage = Volts.of(flMotor.getAppliedOutput() * flMotor.getBusVoltage());
        inputs.rightVoltage = Volts.of(frMotor.getAppliedOutput() * frMotor.getBusVoltage());

        // Positions
        inputs.leftPosition = Meters.of(flEncoder.getPosition());
        inputs.rightPosition = Meters.of(frEncoder.getPosition());

        // Current
        inputs.leftCurrent = Amps.of(flMotor.getOutputCurrent());
        inputs.rightCurrent = Amps.of(frMotor.getOutputCurrent());

        // Temperature
        inputs.leftTemperature = Celsius.of(flMotor.getMotorTemperature());
        inputs.rightTemperature = Celsius.of(frMotor.getMotorTemperature());

        // Velocity
        inputs.leftVelocity = MetersPerSecond.of(flEncoder.getVelocity());
        inputs.rightVelocity = MetersPerSecond.of(frEncoder.getVelocity());
    }

    @Override
    public void setLeftVoltage(Voltage volts) {
        flMotor.setVoltage(volts);
    }

    @Override
    public void setRightVoltage(Voltage volts) {
        frMotor.setVoltage(volts);
    }

    @Override
    public void setLeftSpeed(LinearVelocity velocity) {
        flController.setReference(velocity.in(MetersPerSecond), ControlType.kVelocity);
    }

    @Override
    public void setRightSpeed(LinearVelocity velocity) {
        frController.setReference(velocity.in(MetersPerSecond), ControlType.kVelocity);
    }
}