package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import frc.robot.util.AdjustableValues;

public class DrivetrainIOSim implements DrivetrainIO {
    private DifferentialDrivetrainSim driveSim;

    private PIDController leftController;
    private PIDController rightController;

    private double leftVolts;
    private double rightVolts;

    private boolean openLoop = true;
    
    public DrivetrainIOSim() {
        driveSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDoubleNEOPerSide, KitbotGearing.k8p45, KitbotWheelSize.kEightInch, null);

        leftController = new PIDController(
            AdjustableValues.getNumber("Drive_LeftkP"),
            AdjustableValues.getNumber("Drive_LeftkI"),
            AdjustableValues.getNumber("Drive_LeftkD"));

        rightController = new PIDController(
            AdjustableValues.getNumber("Drive_RightkP"),
            AdjustableValues.getNumber("Drive_RightkI"),
            AdjustableValues.getNumber("Drive_RightkD"));
    }

    /**
     * This function updates the logged inputs.
     * It should be placed in a periodic function somewhere.
     * 
     * @param inputs An instance of the class that contains the inputs that need to be logged.
     */
    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        // Checking if PID values have changed
        if (AdjustableValues.hasChanged("Drive_LeftkP")) leftController.setP(AdjustableValues.getNumber("Drive_LeftkP"));
        if (AdjustableValues.hasChanged("Drive_LeftkI")) leftController.setI(AdjustableValues.getNumber("Drive_LeftkI"));
        if (AdjustableValues.hasChanged("Drive_LeftkD")) leftController.setD(AdjustableValues.getNumber("Drive_LeftkD"));

        if (AdjustableValues.hasChanged("Drive_RightkP")) rightController.setP(AdjustableValues.getNumber("Drive_RightkP"));
        if (AdjustableValues.hasChanged("Drive_RightkI")) rightController.setI(AdjustableValues.getNumber("Drive_RightkI"));
        if (AdjustableValues.hasChanged("Drive_RightkD")) rightController.setD(AdjustableValues.getNumber("Drive_RightkD"));

        // Driving the simulated drivetrain if the robot isn't in open loop control
        if (!openLoop) {
            leftVolts = leftController.calculate(driveSim.getLeftPositionMeters()) + AdjustableValues.getNumber("Drive_LeftkFF");
            rightVolts = rightController.calculate(driveSim.getRightPositionMeters()) + AdjustableValues.getNumber("Drive_RightkFF");
        }

        driveSim.setInputs(leftVolts, rightVolts);

        // Updating the simulated values
        driveSim.update(0.02);

        // Updating the inputs
        // Current
        inputs.flCurrent = Amps.of(driveSim.getLeftCurrentDrawAmps());
        inputs.frCurrent = Amps.of(driveSim.getRightCurrentDrawAmps());

        // Position
        inputs.flPosition = Meters.of(driveSim.getLeftPositionMeters());
        inputs.frPosition = Meters.of(driveSim.getRightPositionMeters());

        // Velocity
        inputs.flVelocity = MetersPerSecond.of(driveSim.getLeftVelocityMetersPerSecond());
        inputs.frVelocity = MetersPerSecond.of(driveSim.getRightVelocityMetersPerSecond());

        // Voltage
        inputs.flVoltage = Volts.of(leftVolts);
        inputs.frVoltage = Volts.of(rightVolts);
    }

    @Override
    public void setLeftVoltage(Voltage volts) {
        openLoop = true;

        leftVolts = volts.in(Volts);
    }

    @Override
    public void setRightVoltage(Voltage volts) {
        openLoop = true;

        rightVolts = volts.in(Volts);
    }

    @Override
    public void setLeftSpeed(LinearVelocity velocity) {
        openLoop = false;

        leftController.setSetpoint(velocity.in(MetersPerSecond));
    }

    @Override
    public void setRightSpeed(LinearVelocity velocity) {
        openLoop = false;

        rightController.setSetpoint(velocity.in(MetersPerSecond));        
    }
}