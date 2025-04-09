package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.drivetrain.Commands.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.Commands.*;

public class RobotContainer {
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private CommandJoystick controller = new CommandJoystick(0);

    // Subsystems
    private Drivetrain drivetrain;
    private Shooter shooter;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        // Initializing subsystems
        if (RobotBase.isReal()) {
            drivetrain = new Drivetrain(new DrivetrainIOTalonSRX(RobotMap.DRIVE_FrontLeftId, RobotMap.DRIVE_FrontRightId, RobotMap.DRIVE_BackLeftId, RobotMap.DRIVE_BackRightId));
            shooter = new Shooter(new ShooterIOSparkMax(RobotMap.SHOOTER_FeedId, RobotMap.SHOOTER_LaunchId));
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
        drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, controller::getY, controller::getZ));

        controller.button(1).whileTrue(new SetFeedPercent(shooter, 1));
        controller.button(3).whileTrue(new SetLaunchPercent(shooter, -1)).whileTrue(new SetFeedPercent(shooter, -1));
        controller.button(2).toggleOnTrue(new SetLaunchPercent(shooter, 1));
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