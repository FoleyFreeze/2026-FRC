// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.auto.ChoreoAutos;
import frc.robot.commands.DriveTuning;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOHardware;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.spindexter.Spindexter;
import frc.robot.subsystems.spindexter.SpindexterIO;
import frc.robot.subsystems.spindexter.SpindexterIOHardware;
import frc.robot.subsystems.spindexter.SpindexterIOSim;
import frc.robot.subsystems.stats.StatsSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
    public final Spindexter spindexter;
    public final Shooter shooter;
    public final Intake intake;
    public final Climber climber;
    public final StatsSubsystem stats;

    public final ChoreoAutos chAutos;

    private SwerveDriveSimulation driveSimulation;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        stats = new StatsSubsystem();
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
                // a CANcoder
                drive =
                        new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight),
                                (pose) -> {});

                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                new VisionIOLimelight("camera0Name", drive::getRotation),
                                new VisionIOLimelight("camera1Name", drive::getRotation));

                spindexter = new Spindexter(new SpindexterIOHardware(), this);

                shooter = new Shooter(new ShooterIOHardware(), this);

                intake = new Intake(new IntakeIOHardware());

                climber = new Climber(new ClimberIOHardware(), this);

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                driveSimulation =
                        new SwerveDriveSimulation(
                                Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive =
                        new Drive(
                                new GyroIOSim(driveSimulation.getGyroSimulation()),
                                new ModuleIOSim(driveSimulation.getModules()[0]),
                                new ModuleIOSim(driveSimulation.getModules()[1]),
                                new ModuleIOSim(driveSimulation.getModules()[2]),
                                new ModuleIOSim(driveSimulation.getModules()[3]),
                                driveSimulation::setSimulationWorldPose);

                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                new VisionIOPhotonVisionSim(
                                        "camera0Name",
                                        new Transform3d(),
                                        driveSimulation::getSimulatedDriveTrainPose),
                                new VisionIOPhotonVisionSim(
                                        "camera1Name",
                                        new Transform3d(),
                                        driveSimulation::getSimulatedDriveTrainPose));

                SpindexterIOSim spinSim = new SpindexterIOSim();
                spindexter = new Spindexter(spinSim, this);

                IntakeIOSim iis = new IntakeIOSim(driveSimulation);
                intake = new Intake(iis);

                ShooterIOSim shootSim = new ShooterIOSim(iis, driveSimulation, spinSim);
                shooter = new Shooter(shootSim, this);
                shootSim.registerShooter(shooter);

                climber = new Climber(new ClimberIOSim(), this);
                break;

            default:
                // Replayed robot, disable IO implementations
                drive =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                (pose) -> {});

                // (Use same number of dummy implementations as the real robot)
                vision =
                        new Vision(
                                drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

                spindexter = new Spindexter(new SpindexterIO() {}, this);

                shooter = new Shooter(new ShooterIO() {}, this);

                intake = new Intake(new IntakeIO() {});

                climber = new Climber(new ClimberIO() {}, this);

                break;
        }

        chAutos = new ChoreoAutos(this);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        autoChooser.addOption("TestAutoLeft", chAutos.loadTraj("TestAutoLeft"));

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                DriveTuning.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveTuning.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        ConfigButtons.config(this);
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

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
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
