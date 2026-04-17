// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Mode;
import frc.robot.auto.ChoreoAutos;
import frc.robot.auto.PathAutos;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveTuning;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.fuelvision.FuelVision;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.spindexter.Spindexter;
import frc.robot.subsystems.spindexter.SpindexterIOSim;
import frc.robot.subsystems.stats.StatsSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.MatchPhaseUtil;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    public final Drive drive;
    public final Vision vision;
    public FuelVision fuelVision;
    public final Spindexter spindexter;
    public final Shooter shooter;
    public final Intake intake;
    public final Climber climber;
    public final StatsSubsystem stats;
    public final MatchPhaseUtil matchPhaseUtil;
    public final Led led;

    public final ChoreoAutos chAutos;
    public final PathAutos pathAutos;

    public SwerveDriveSimulation driveSimulation;

    // Dashboard inputs
    public final LoggedDashboardChooser<Command> autoChooser;

    private static RobotContainer instance;

    public static RobotContainer getInstance() {
        return instance;
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        instance = this;
        matchPhaseUtil = new MatchPhaseUtil();
        SpindexterIOSim spinSim = null;
        IntakeIOSim iis = null;
        ShooterIOSim shootSim = null;
        if (Constants.currentMode == Mode.SIM) {
            driveSimulation =
                    new SwerveDriveSimulation(
                            Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
            spinSim = new SpindexterIOSim();
            iis = new IntakeIOSim(driveSimulation);
            shootSim = new ShooterIOSim(iis, driveSimulation, spinSim);
        } else {
            driveSimulation = null;
        }

        stats = new StatsSubsystem(this);
        drive = Drive.create(this, driveSimulation);
        fuelVision = FuelVision.create(this);
        spindexter = Spindexter.create(this, spinSim);
        shooter = Shooter.create(this, shootSim);
        shooter.zeroTurret();
        vision = Vision.create(this, drive, shooter, driveSimulation);
        intake = Intake.create(this, iis);
        climber = Climber.create(this);
        led = new Led(this);

        if (Constants.currentMode == Mode.SIM) {
            shootSim.registerShooter(shooter);
        }

        chAutos = new ChoreoAutos(this);
        pathAutos = new PathAutos(this);

        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        // Set up auto routines
        pathAutos.buildAutos(autoChooser);
        // chAutos.buildAutos(autoChooser);

        // Set up SysId routines
        DriveTuning.buildSysIdAutos(autoChooser, this);

        ConfigButtons.config(this);
        DriveCommands.initDriveCommands();

        CommandScheduler.getInstance().setPeriod(0.2);
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    LoggedMechanism2d mech;
    LoggedMechanismRoot2d mechRootI;
    LoggedMechanismRoot2d mechRootT;
    LoggedMechanismLigament2d intakeMech;
    LoggedMechanismLigament2d turretMechOffset;
    LoggedMechanismLigament2d turretHoodMech;

    public void simInit() {
        mech = new LoggedMechanism2d(0, 0);
        mechRootI = mech.getRoot("IntakeRoot", Units.inchesToMeters(16.5), 0);
        intakeMech =
                mechRootI.append(
                        new LoggedMechanismLigament2d("Intake", Units.inchesToMeters(10), 0));

        mechRootT = mech.getRoot("ShootRoot", Constants.shooterLocOnBot.getX(), 0);
        turretMechOffset =
                mechRootT.append(
                        new LoggedMechanismLigament2d(
                                "TurretOffset",
                                Units.inchesToMeters(18),
                                90,
                                10,
                                new Color8Bit(110, 110, 110)));
        turretHoodMech =
                turretMechOffset.append(
                        new LoggedMechanismLigament2d("Hood", Units.inchesToMeters(6), -45));
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3.9, 7.5, Rotation2d.k180deg));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput(
                "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Fuel",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));

        intakeMech.setAngle(intake.getAngle());
        turretHoodMech.setAngle(-shooter.getHoodAngle());

        Logger.recordOutput("Mech", mech);
    }
}
