package frc.robot.Shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    // Motors
    private SparkMax launchMotor;
    private SparkMax feedMotor;

    // Encoders
    private RelativeEncoder launchEncoder;
    private RelativeEncoder feedEncoder;

    // PID Controllers
    private SparkClosedLoopController launchPID;
    private SparkClosedLoopController feedPID;

    // Logged Vars
    private double expectedFeedSpeed;
    private double expectedLaunchSpeed;

    // A common instance of the shooter class so that I don't have to outright initialize it anywhere.
    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null) instance = new Shooter();

        return instance;
    }

    public Shooter() {
        // Getting motors
        launchMotor = new SparkMax(ShooterConstants.launchID, MotorType.kBrushless);
        feedMotor = new SparkMax(ShooterConstants.feedID, MotorType.kBrushless);

        // Initializing the config object
        SparkMaxConfig config = new SparkMaxConfig();

        // Adding configs
        config.idleMode(IdleMode.kCoast);
        config.inverted(true);

        // Applying launch PIDs
        config.closedLoop.p(ShooterConstants.launchP);
        config.closedLoop.i(ShooterConstants.launchI);
        config.closedLoop.d(ShooterConstants.launchD);
        config.closedLoop.velocityFF(ShooterConstants.launchFF);

        // Configuring launch motor
        launchMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Applying feed PIDs
        config.closedLoop.p(ShooterConstants.feedP);
        config.closedLoop.i(ShooterConstants.feedI);
        config.closedLoop.d(ShooterConstants.feedD);
        config.closedLoop.velocityFF(ShooterConstants.feedFF);

        // Configuring feed motor
        feedMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Getting the encoders
        launchEncoder = launchMotor.getEncoder();
        feedEncoder = feedMotor.getEncoder();

        // Getting the PID controllers
        launchPID = launchMotor.getClosedLoopController();
        feedPID = feedMotor.getClosedLoopController();
    }

    /** This function runs every tick that the class has been initialized. */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("/Shooter/Real_Feed_RPM", getFeedSpeed());
        SmartDashboard.putNumber("/Shooter/Expected_Feed_RPM", expectedFeedSpeed);
        SmartDashboard.putNumber("/Shooter/Real_Launch_RPM", getLaunchSpeed());
        SmartDashboard.putNumber("/Shooter/Expected_Launch_RPM", expectedLaunchSpeed);
    }

    /** Gets the speed of the feed wheel. */
    public double getFeedSpeed() {
        return feedEncoder.getVelocity();
    }

    /** Gets the speed of the launch wheel. */
    public double getLaunchSpeed() {
        return launchEncoder.getVelocity();
    }

    /** Sets the speed of the feed wheel. */
    public void setFeedSpeed(double speed) {
        expectedFeedSpeed = speed * ShooterConstants.maxFeedSpeed.in(Rotations.per(Minute));
        
        feedPID.setReference(expectedFeedSpeed, ControlType.kVelocity);
    }

    /** Sets the speed of the launch wheel. */
    public void setLaunchSpeed(double speed) {
        expectedLaunchSpeed = speed * ShooterConstants.maxLaunchSpeed.in(Rotations.per(Minute));
        
        launchPID.setReference(expectedLaunchSpeed, ControlType.kVelocity);
    }
}