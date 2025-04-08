package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.drivetrain.Commands.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.Commands.*;

public class RobotContainer {
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private CommandXboxController controller = new CommandXboxController(0);

    // Subsystems
    private Drivetrain drivetrain;
    private Shooter shooter;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        // Initializing subsystems
        if (RobotBase.isReal()) {
            drivetrain = new Drivetrain(new DrivetrainIOTalonSRX(DriveConstants.frontLeftID, DriveConstants.frontRightID, DriveConstants.backLeftID, DriveConstants.backRightID));
            shooter = new Shooter(new ShooterIOSparkMax(ShooterConstants.feedID, ShooterConstants.launchID));
        } else {
            drivetrain = new Drivetrain(new DrivetrainIOSim());
            shooter = new Shooter(new ShooterIOSim());
        }
        
        // Configuring button/trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, controller::getLeftY, controller::getRightX));

        controller.a().whileTrue(new RunFeed(shooter, 1));
        controller.b().whileTrue(new RunShooter(shooter, -1)).whileTrue(new RunFeed(shooter, -1));
        controller.y().toggleOnTrue(new RunShooter(shooter, 1));
    }

    /**
     * Use this to pass an autonomous command to the {@link Robot} class.
     *
     * @return The command to run in Autonomous mode.
     */
    public Command getAutoCommand() {
        return new PrintCommand("No auto lol");
    }
}