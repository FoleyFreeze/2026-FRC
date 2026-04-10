// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LimelightHelpers;
import java.lang.reflect.Field;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private Command prevAuto;
    private RobotContainer robotContainer;
    private boolean lastPdhStatus = true;
    public LoggedPowerDistribution pdh;
    private Timer canivoreUpdateTimer = new Timer();

    public Robot() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(0.1);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to disable loop overrun warnings", false);
        }
        CommandScheduler.getInstance().setPeriod(0.1);

        RobotController.setBrownoutVoltage(6.0);

        // Start AdvantageKit logger
        LoggedPowerDistribution.getInstance(1, ModuleType.kRev);
        Logger.start();
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        canivoreUpdateTimer.start();
    }

    /** This function is called periodically during all modes. */
    double prevTime = 0;

    @Override
    public void robotPeriodic() {

        // Optionally switch the thread to high priority to improve loop
        // timing (see the template project documentation for details)
        // Threads.setCurrentThreadPriority(true, 99);

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        robotContainer.matchPhaseUtil.matchPhaseUtil();
        CommandScheduler.getInstance().run();

        Logger.recordOutput(
                "Vision/TurretHB", LimelightHelpers.getHeartbeat(VisionConstants.camera0Name));
        Logger.recordOutput(
                "Vision/LeftHB", LimelightHelpers.getHeartbeat(VisionConstants.camera1Name));

        if (Constants.currentMode == Mode.REAL && canivoreUpdateTimer.advanceIfElapsed(0.2)) {
            var canStatus = TunerConstants.kCANBus.getStatus();
            Logger.recordOutput("CANivore/Status", canStatus.Status.getName());
            Logger.recordOutput("CANivore/Utilization", canStatus.BusUtilization);
            Logger.recordOutput("CANivore/OffCount", canStatus.BusOffCount);
            Logger.recordOutput("CANivore/TxFullCount", canStatus.TxFullCount);
            Logger.recordOutput("CANivore/ReceiveErrors", canStatus.REC);
            Logger.recordOutput("CANivore/TransmitErrors", canStatus.TEC);
        }

        // Return to non-RT thread priority (do not modify the first argument)
        // Threads.setCurrentThreadPriority(false, 10);
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        robotContainer.resetSimulationField();
        Constants.isEnabled = false;
        Constants.isAuto = false;

        // robotContainer.drive.setTurnBrakeMode(false);
    }

    boolean wasTeleop = false;
    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        if (DriverStation.isFMSAttached()) {
            if (wasTeleop) {
                wasTeleop = false;
                LimelightHelpers.triggerRewindCapture(VisionConstants.camera0Name, 165);
                LimelightHelpers.triggerRewindCapture(VisionConstants.camera1Name, 165);
            }

            if (lastPdhStatus) {

            } else {
                // pdh.setSwitchableChannel(true);
                lastPdhStatus = true;
            }
        } else {
            if (lastPdhStatus) {
                // pdh.setSwitchableChannel(false);
                lastPdhStatus = false;
            } else {

            }
        }
        wasTeleop = false;

        // rezero to the start point of the auton
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (prevAuto != autonomousCommand) {
            System.out.println("Updated auto");
            Logger.recordOutput(
                    "Auton/Selected",
                    DriverStation.getAlliance().toString()
                            + ", "
                            + robotContainer.autoChooser.getSendableChooser().getSelected());
            if (robotContainer.pathAutos.pathMap.containsKey(autonomousCommand)) {
                // this should zero to auton initial state
                robotContainer.pathAutos.pathMap.get(autonomousCommand).run();
            } else {
                System.out.println("No auton zero configured");
            }
            prevAuto = autonomousCommand;
        }
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // pdh.setSwitchableChannel(true);
        lastPdhStatus = true;
        Constants.isEnabled = true;
        Constants.isAuto = true;

        robotContainer.drive.setBrakeMode(true);

        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        // pdh.setSwitchableChannel(true);
        lastPdhStatus = true;
        Constants.isEnabled = true;
        Constants.isAuto = false;
        wasTeleop = true;

        robotContainer.drive.setBrakeMode(false);

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
        robotContainer.simInit();
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        robotContainer.updateSimulation();
    }
}
