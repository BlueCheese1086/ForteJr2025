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
    private SparkMax launch;
    private SparkMax feed;

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
        // Switching IDs temporarily
        launch = new SparkMax(ShooterConstants.feedID, MotorType.kBrushless);
        feed = new SparkMax(ShooterConstants.launchID, MotorType.kBrushless);

        // Initializing the config object
        SparkMaxConfig config = new SparkMaxConfig();

        // Adding configs
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);

        // Configuring launch motor
        launch.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configuring feed motor
        feed.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** This function runs every tick that the class has been initialized. */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("/Shooter/Real_Feed_Speed", feed.get());
        SmartDashboard.putNumber("/Shooter/Expected_Feed_Speed", expectedFeedSpeed);
        SmartDashboard.putNumber("/Shooter/Real_Launch_Speed", launch.get());
        SmartDashboard.putNumber("/Shooter/Expected_Launch_Speed", expectedLaunchSpeed);
    }

    /** Gets the speed of the feed wheel. */
    public double getFeedSpeed() {
        return feed.get();
    }

    /** Gets the speed of the launch wheel. */
    public double getLaunchSpeed() {
        return launch.get();
    }

    /** Sets the speed of the feed wheel. */
    public void setFeedSpeed(double speed) {
        expectedFeedSpeed = speed;
        feed.set(speed);
    }

    /** Sets the speed of the launch wheel. */
    public void setLaunchSpeed(double speed) {
        expectedLaunchSpeed = speed;
        launch.set(speed);
    }
}