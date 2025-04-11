package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {
    private DrivetrainIO drivetrainIO;
    private DrivetrainIOInputsAutoLogged inputs;

    // Kinematics
    private DifferentialDriveKinematics kinematics;

    // Odometry
    private DifferentialDrivePoseEstimator poseEstimator;

    private Rotation2d heading = new Rotation2d();

    /** Creates a new Drivetrain. */
    public Drivetrain(DrivetrainIO drivetrainIO) {
        this.drivetrainIO = drivetrainIO;

        inputs = new DrivetrainIOInputsAutoLogged();
        
        // Pushing a warning to SmartDashboard if TalonSRXs are in use
        // They don't have built-in encoders and cannot run closed loop
        if (drivetrainIO instanceof DrivetrainIOTalonSRX) {
            SmartDashboard.putString("/Warnings/OpenLoop", "Running open loop!  Many logged values will be zero.");
        }

        // Initializing the kinematics
        kinematics = new DifferentialDriveKinematics(DriveConstants.robotWidth);

        // Initializing the pose estimator
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, getAngle(), getLeftPosition().in(Meters), getRightPosition().in(Meters), new Pose2d());

        // Configuring Pathplanner
        // Flips the path if on Red alliance.  Pathplanner builds their paths for Blue alliance.
        // Defaults to flipped because simulated robots start on the Red alliance.
        AutoBuilder.configure(
            this::getPose, 
            this::setPose, 
            this::getChassisSpeeds, 
            this::closedLoop, 
            new PPLTVController(0.02, DriveConstants.maxDriveSpeed.in(MetersPerSecond)),
            new RobotConfig(DriveConstants.mass, DriveConstants.momentOfInertia, 
                new ModuleConfig(DriveConstants.wheelRadius,
                    DriveConstants.maxDriveSpeed, 1,
                    DCMotor.getCIM(2),
                    DriveConstants.gearRatio, DriveConstants.driveCurrentLimit, 2),
                DriveConstants.robotWidth),
            () -> (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red),
            this);
    }

    @Override
    public void periodic() {
        drivetrainIO.updateInputs(inputs);

        Logger.processInputs("/RealOutputs/Drivetrain", inputs);

        // Estimating heading from IO inputs
        heading.plus(new Rotation2d(kinematics.toTwist2d(inputs.flPosition.in(Meters), inputs.frPosition.in(Meters)).dtheta));

        poseEstimator.update(getAngle(), getPosition());
    }

    public Rotation2d getAngle() {
        return heading;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d newPose) {
        poseEstimator.resetPosition(getAngle(), getPosition(), newPose);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getSpeeds());
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    public DifferentialDriveWheelPositions getPosition() {
        return new DifferentialDriveWheelPositions(getLeftPosition(), getRightPosition());
    }

    public Distance getLeftPosition() {
        return inputs.flPosition;
    }

    public LinearVelocity getLeftVelocity() {
        return inputs.flVelocity;
    }

    public Distance getRightPosition() {
        return inputs.frPosition;
    }

    public LinearVelocity getRightVelocity() {
        return inputs.frVelocity;
    }

    public void arcadeDrive(double xSpeed, double zRotate) {
        drivetrainIO.setLeftVoltage(Volts.of((xSpeed - zRotate) * RobotController.getInputVoltage()));
        drivetrainIO.setRightVoltage(Volts.of((xSpeed + zRotate) * RobotController.getInputVoltage()));
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        drivetrainIO.setLeftVoltage(Volts.of(leftSpeed * RobotController.getInputVoltage()));
        drivetrainIO.setRightVoltage(Volts.of(rightSpeed * RobotController.getInputVoltage()));
    }

    public void closedLoop(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds newSpeeds = kinematics.toWheelSpeeds(speeds);
        
        drivetrainIO.setLeftSpeed(MetersPerSecond.of(newSpeeds.leftMetersPerSecond));
        drivetrainIO.setRightSpeed(MetersPerSecond.of(newSpeeds.rightMetersPerSecond));
    }
}