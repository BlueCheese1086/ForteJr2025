package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.Constants.DriveConstants;

public class DrivetrainIOTalonSRX implements DrivetrainIO {
    private TalonSRX flMotor;
    private TalonSRX frMotor;
    private TalonSRX blMotor;
    private TalonSRX brMotor;

    // Alert for when the user uses closed loop control on the brushed motor controllers.
    private Alert noClosedLoop = new Alert("TalonSRX unable to use closed loop control.  Rolling back to voltage output.", Alert.AlertType.kWarning);

    /**
     * TalonSRXs are brushed controllers, and do not have a built-in encoder.
     * This results in robots without external encoders to not know their wheel positions or velocity.
     */
    public DrivetrainIOTalonSRX(int frontLeftID, int frontRightID, int backLeftID, int backRightID) {
        // Initializing the motors
        flMotor = new TalonSRX(frontLeftID);
        frMotor = new TalonSRX(frontRightID);
        blMotor = new TalonSRX(backLeftID);
        brMotor = new TalonSRX(backRightID);

        // Resetting each TalonSRX's settings.
        flMotor.configFactoryDefault();
        frMotor.configFactoryDefault();
        blMotor.configFactoryDefault();
        brMotor.configFactoryDefault();

        // Inverting the motors
        flMotor.setInverted(true);
        frMotor.setInverted(false);
        blMotor.setInverted(true);
        brMotor.setInverted(false);

        // Putting the motors into brake mode.
        flMotor.setNeutralMode(NeutralMode.Brake);
        frMotor.setNeutralMode(NeutralMode.Brake);
        blMotor.setNeutralMode(NeutralMode.Brake);
        brMotor.setNeutralMode(NeutralMode.Brake);

        // Having the back motors follow the front motors.
        blMotor.follow(flMotor);
        brMotor.follow(frMotor);
    }

    /**
     * This function updates the logged inputs.
     * It should be placed in a periodic function somewhere.
     * 
     * @param inputs An instance of the class that contains the inputs that need to be logged.
     */
    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        // Updating IOInputs
        // Current
        inputs.flCurrent = Amps.of(flMotor.getStatorCurrent());
        inputs.frCurrent = Amps.of(frMotor.getStatorCurrent());
        inputs.blCurrent = Amps.of(blMotor.getStatorCurrent());
        inputs.brCurrent = Amps.of(brMotor.getStatorCurrent());

        // Temperature
        inputs.flTemperature = Celsius.of(flMotor.getTemperature());
        inputs.frTemperature = Celsius.of(frMotor.getTemperature());
        inputs.blTemperature = Celsius.of(blMotor.getTemperature());
        inputs.brTemperature = Celsius.of(brMotor.getTemperature());

        // Voltage
        inputs.flVoltage = Volts.of(flMotor.getMotorOutputVoltage());
        inputs.frVoltage = Volts.of(frMotor.getMotorOutputVoltage());
        inputs.blVoltage = Volts.of(blMotor.getMotorOutputVoltage());
        inputs.brVoltage = Volts.of(brMotor.getMotorOutputVoltage());
    }

    @Override
    public void setLeftVoltage(Voltage volts) {
        noClosedLoop.set(false);

        flMotor.set(ControlMode.PercentOutput, volts.in(Volts) / flMotor.getBusVoltage());
    }

    @Override
    public void setRightVoltage(Voltage volts) {
        noClosedLoop.set(false);

        frMotor.set(ControlMode.PercentOutput, volts.in(Volts) / frMotor.getBusVoltage());
    }

    @Override
    public void setLeftSpeed(LinearVelocity velocity) {
        noClosedLoop.set(true);
        
        flMotor.set(ControlMode.PercentOutput, velocity.div(DriveConstants.maxDriveSpeed).magnitude());
    }

    @Override
    public void setRightSpeed(LinearVelocity velocity) {
        noClosedLoop.set(true);

        frMotor.set(ControlMode.PercentOutput, velocity.div(DriveConstants.maxDriveSpeed).magnitude());
    }
}