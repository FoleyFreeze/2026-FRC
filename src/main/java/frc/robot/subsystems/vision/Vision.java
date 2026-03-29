// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    RobotContainer r;

    public static final boolean isDisabled = false;

    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    public static Vision create(
            RobotContainer r, Drive drive, Shooter shoot, SwerveDriveSimulation driveSimulation) {
        if (isDisabled) {
            return new Vision(r, drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        }

        switch (Constants.currentMode) {
            case REAL:
                // TODO: add shoot::getTurretAngle to turret camera (cam0) when turret is installed
                return new Vision(
                        r,
                        drive::addVisionMeasurement,
                        new VisionIOLimelight(
                                r,
                                VisionConstants.camera0Name,
                                drive::getRotation,
                                VisionConstants.robotToCamera0,
                                r.shooter::getTurretAngle),
                        new VisionIOLimelight(
                                r,
                                VisionConstants.camera1Name,
                                drive::getRotation,
                                VisionConstants.robotToCamera1));

            case SIM:
                return new Vision(
                        r,
                        drive::addVisionMeasurement,
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera0Name,
                                new Transform3d(),
                                driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera1Name,
                                new Transform3d(),
                                driveSimulation::getSimulatedDriveTrainPose));

            default:
                return new Vision(
                        r, drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        }
    }

    public Vision(RobotContainer r, VisionConsumer consumer, VisionIO... io) {
        this.r = r;
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                    new Alert(
                            "Vision camera " + Integer.toString(i) + " is disconnected.",
                            AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    public int validImages = 0;
    int turretCamAcceptedPose = 0;
    Translation2d lastTagPose = new Translation2d();
    Translation2d last2TagPose = new Translation2d();

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        turretCamAcceptedPose--; // decrement every loop

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose =
                        observation.tagCount() == 0 // Must have at least one tag
                                || (observation.tagCount() == 1
                                        && observation.ambiguity()
                                                > maxAmbiguity) // Cannot be high ambiguity
                                || Math.abs(observation.pose().getZ())
                                        > maxZError // Must have realistic Z coordinate
                                // reject all megatag1 poses when enabled
                                || DriverStation.isEnabled()
                                        && (observation.type() == PoseObservationType.MEGATAG_1
                                                || observation.type()
                                                        == PoseObservationType.MEGATAG_1_T)
                                // if we have both cameras reporting a pose, only take the turret
                                // cam
                                || cameraIndex == 1 && turretCamAcceptedPose > 0
                                // Must be within the field boundaries
                                || observation.pose().getX() < 0.0
                                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                                || observation.pose().getY() < 0.0
                                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());

                    // record if turret cam accepted a pose this loop
                    if (cameraIndex == 0) {
                        turretCamAcceptedPose = 3;
                    }
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor =
                        observation.averageTagDistance()
                                * observation.averageTagDistance()
                                / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;

                // only accept angles from multi tag images
                double angularStdDev;
                if (observation.tagCount() > 1) {
                    angularStdDev = angularStdDevBaseline * stdDevFactor;
                } else {
                    angularStdDev = Double.POSITIVE_INFINITY;
                }

                switch (observation.type()) {
                    case MEGATAG_2:
                        linearStdDev *= linearStdDevMegatag2Factor;
                        angularStdDev *= angularStdDevMegatag2Factor;
                    case MEGATAG_1:
                        break;

                        // turret tags are weighted
                    case MEGATAG_2_T:
                        linearStdDev *= linearStdDevMegatag2Factor;
                        angularStdDev *= angularStdDevMegatag2Factor;
                    case MEGATAG_1_T:
                        double extraForTurretRotate =
                                1
                                        + Math.abs(
                                                        Units.degreesToRotations(
                                                                r.shooter.inputs.turretVelocity))
                                                * 2;
                        linearStdDev *= extraForTurretRotate;
                        angularStdDev *= extraForTurretRotate;
                        break;

                    default:
                }
                if (observation.type() == PoseObservationType.MEGATAG_2) {}

                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                // Send vision observation
                consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

                lastTagPose = r.drive.getPose().getTranslation();
                if (observation.tagCount() > 1) {
                    last2TagPose = r.drive.getPose().getTranslation();
                }
            }

            // Log camera metadata
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[0]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);

            validImages = allRobotPoses.size();

            if (r.drive.getPose().getTranslation().getDistance(lastTagPose) < 1) {
                Logger.recordOutput("Vision/hasRecentTag", true);
            } else {
                Logger.recordOutput("Vision/hasRecentTag", false);
            }

            if (r.drive.getPose().getTranslation().getDistance(last2TagPose) < 1) {
                Logger.recordOutput("Vision/hasRecent2Tag", true);
            } else {
                Logger.recordOutput("Vision/hasRecent2Tag", false);
            }
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
