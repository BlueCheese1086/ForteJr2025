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
    private RelativeEncoder blEncoder;
    private RelativeEncoder brEncoder;

    private SparkClosedLoopController flController;
    private SparkClosedLoopController frController;

    private SparkMaxConfig leftConfig = new SparkMaxConfig();
    private SparkMaxConfig rightConfig = new SparkMaxConfig();

    public DrivetrainIOSparkMax(int frontLeftID, int frontRightID, int backLeftID, int backRightID) {
        // Initializing the motors
        flMotor = new SparkMax(frontLeftID,  MotorType.kBrushless);
        frMotor = new SparkMax(frontRightID, MotorType.kBrushless);
        blMotor = new SparkMax(backLeftID,   MotorType.kBrushless);
        brMotor = new SparkMax(backRightID,  MotorType.kBrushless);

        // Adjusting the config objects
        leftConfig.idleMode(IdleMode.kBrake);
        leftConfig.inverted(true);
        leftConfig.closedLoop.p(AdjustableValues.getNumber("Drive_LeftkP"));
        leftConfig.closedLoop.i(AdjustableValues.getNumber("Drive_LeftkI"));
        leftConfig.closedLoop.d(AdjustableValues.getNumber("Drive_LeftkD"));
        leftConfig.closedLoop.velocityFF(AdjustableValues.getNumber("Drive_LeftkFF"));
        flMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        leftConfig.follow(flMotor);
        blMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        leftConfig.disableFollowerMode();

        rightConfig.idleMode(IdleMode.kBrake);
        rightConfig.closedLoop.p(AdjustableValues.getNumber("Drive_RightkP"));
        rightConfig.closedLoop.i(AdjustableValues.getNumber("Drive_RightkI"));
        rightConfig.closedLoop.d(AdjustableValues.getNumber("Drive_RightkD"));
        rightConfig.closedLoop.velocityFF(AdjustableValues.getNumber("Drive_RightkFF"));
        frMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        rightConfig.follow(frMotor);
        brMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        rightConfig.disableFollowerMode();
        
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
        boolean changed = false;

        if (AdjustableValues.hasChanged("Drive_LeftkP")) {
            leftConfig.closedLoop.p(AdjustableValues.getNumber("Drive_LeftkP"));

            changed = true;
        }

        if (AdjustableValues.hasChanged("Drive_LeftkI")) {
            leftConfig.closedLoop.i(AdjustableValues.getNumber("Drive_LeftkI"));

            changed = true;
        }

        if (AdjustableValues.hasChanged("Drive_LeftkD")) {
            leftConfig.closedLoop.d(AdjustableValues.getNumber("Drive_LeftkD"));

            changed = true;
        }

        if (AdjustableValues.hasChanged("Drive_LeftkFF")) {
            leftConfig.closedLoop.velocityFF(AdjustableValues.getNumber("Drive_LeftkFF"));

            changed = true;
        }

        if (changed) {
            flMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            leftConfig.follow(flMotor);
            blMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            leftConfig.disableFollowerMode();
        }

        changed = false;

        if (AdjustableValues.hasChanged("Drive_RightkP")) {
            rightConfig.closedLoop.p(AdjustableValues.getNumber("Drive_RightkP"));

            changed = true;
        }

        if (AdjustableValues.hasChanged("Drive_RightkI")) {
            rightConfig.closedLoop.i(AdjustableValues.getNumber("Drive_RightkI"));

            changed = true;
        }

        if (AdjustableValues.hasChanged("Drive_RightkD")) {
            rightConfig.closedLoop.d(AdjustableValues.getNumber("Drive_RightkD"));

            changed = true;
        }

        if (AdjustableValues.hasChanged("Drive_RightkFF")) {
            rightConfig.closedLoop.velocityFF(AdjustableValues.getNumber("Drive_RightkFF"));

            changed = true;
        }

        if (changed) {
            frMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
            rightConfig.follow(frMotor);
            brMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            rightConfig.disableFollowerMode();
        }

        // Updating inputs
        // Current
        inputs.flCurrent = Amps.of(flMotor.getOutputCurrent());
        inputs.frCurrent = Amps.of(frMotor.getOutputCurrent());
        inputs.blCurrent = Amps.of(blMotor.getOutputCurrent());
        inputs.brCurrent = Amps.of(brMotor.getOutputCurrent());

        // Position
        inputs.flPosition = Meters.of(flEncoder.getPosition());
        inputs.frPosition = Meters.of(frEncoder.getPosition());
        inputs.blPosition = Meters.of(blEncoder.getPosition());
        inputs.brPosition = Meters.of(brEncoder.getPosition());

        // Temperature
        inputs.flTemperature = Celsius.of(flMotor.getMotorTemperature());
        inputs.frTemperature = Celsius.of(frMotor.getMotorTemperature());
        inputs.blTemperature = Celsius.of(blMotor.getMotorTemperature());
        inputs.brTemperature = Celsius.of(brMotor.getMotorTemperature());

        // Velocity
        inputs.flVelocity = MetersPerSecond.of(flEncoder.getVelocity());
        inputs.frVelocity = MetersPerSecond.of(frEncoder.getVelocity());
        inputs.blVelocity = MetersPerSecond.of(blEncoder.getVelocity());
        inputs.brVelocity = MetersPerSecond.of(brEncoder.getVelocity());

        // Voltage
        inputs.flVoltage = Volts.of(flMotor.getAppliedOutput() * flMotor.getBusVoltage());
        inputs.frVoltage = Volts.of(frMotor.getAppliedOutput() * frMotor.getBusVoltage());
        inputs.blVoltage = Volts.of(blMotor.getAppliedOutput() * blMotor.getBusVoltage());
        inputs.brVoltage = Volts.of(brMotor.getAppliedOutput() * brMotor.getBusVoltage());
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