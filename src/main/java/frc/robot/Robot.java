package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.AdjustableValues;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;
    private Command autonomousCommand;

    /** This function is run when the robot is first started up and should be used for any initialization code. */
    @Override
    public void robotInit() {
        // Adding an NT4Publisher
        Logger.addDataReceiver(new NT4Publisher());

        // Adding a WPILOGWriter if running for real.
        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
        }

        // Adding a replay source if running in sim and as a replay
        if (isSimulation() && Constants.isReplay) {
            Logger.setReplaySource(new WPILOGReader("log.wpilog"));
        }

        // Starting the Logger
        Logger.start();

        // Instantiate the RobotContainer.  This will assign all our button bindings.
        robotContainer = new RobotContainer();

        AdjustableValues.registerNumber("Drive_kP", "/Adjustables/Drive/kP", DriveConstants.kP, "Drive_LeftkP", "Drive_RightkP");
        AdjustableValues.registerNumber("Drive_kI", "/Adjustables/Drive/kI", DriveConstants.kI, "Drive_LeftkI", "Drive_RightkI");
        AdjustableValues.registerNumber("Drive_kD", "/Adjustables/Drive/kD", DriveConstants.kD, "Drive_LeftkD", "Drive_RightkD");
        AdjustableValues.registerNumber("Drive_kFF", "/Adjustables/Drive/kFF", DriveConstants.kFF, "Drive_LeftkFF", "Drive_RightkFF");

        AdjustableValues.registerNumber("Feed_kP", "/Adjustables/Feed/kP", ShooterConstants.feedP);
        AdjustableValues.registerNumber("Feed_kI", "/Adjustables/Feed/kI", ShooterConstants.feedI);
        AdjustableValues.registerNumber("Feed_kD", "/Adjustables/Feed/kD", ShooterConstants.feedD);
        AdjustableValues.registerNumber("Feed_kFF", "/Adjustables/Feed/kFF", ShooterConstants.feedFF);

        AdjustableValues.registerNumber("Launch_kP", "/Adjustables/Launch/kP", ShooterConstants.launchP);
        AdjustableValues.registerNumber("Launch_kI", "/Adjustables/Launch/kI", ShooterConstants.launchI);
        AdjustableValues.registerNumber("Launch_kD", "/Adjustables/Launch/kD", ShooterConstants.launchD);
        AdjustableValues.registerNumber("Launch_kFF", "/Adjustables/Launch/kFF", ShooterConstants.launchFF);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically during Disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This function is called once each time the robot exits Disabled */
    @Override
    public void disabledExit() {}

    /**
     * This function is called once each time the robot enters Autonomous.
     * It starts the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutoCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during Autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /**
     * This function is called once each time the robot exits Autonomous.
     * It stops the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousExit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called once each time the robot enters Teleop. */
    @Override
    public void teleopInit() {}

    /** This function is called periodically during Teleop. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once each time the robot exits Teleop. */
    @Override
    public void teleopExit() {}

    /** This function is called once each time the robot enters Test. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once each time the robot exits Test. */
    @Override
    public void testExit() {}

    /** This function is called once each time the robot enters Simulation. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically during Simulation. */
    @Override
    public void simulationPeriodic() {}
}