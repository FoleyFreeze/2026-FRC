package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.ConfigButtons;
import frc.robot.RobotContainer;
import frc.robot.subsystems.fuelvision.FuelVision;
import frc.robot.subsystems.fuelvision.FuelVision.Zone;
import frc.robot.util.Util2;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class CameraBallGatherCmd extends Command {
    RobotContainer r;
    private Command c; // captured command
    private final PathConstraints globalConstraints = new PathConstraints(2, 1.25, 1, 2);

    private Zone z = Zone.NEUTRAL;
    private double[] rect;
    private Pose2d nextCorner = new Pose2d();
    private int cornerIdx = 0;
    private boolean isCCW = true;
    private Translation2d mid = new Translation2d();

    public CameraBallGatherCmd(RobotContainer r) {
        this.r = r;
        this.addRequirements(r.drive);
        rect = r.fuelVision.fixedRects[z.ordinal()];
    }

    @Override
    public void initialize() {
        c = null;
        try {
            List<Pose2d> pathPoses = r.fuelVision.getFuelPath();
            Logger.recordOutput("FuelVision/FailReason", r.fuelVision.failReason);
            if (pathPoses.isEmpty()) {
                // exit early if path failed
                c = null;
                return;
            }
            nextCorner = pathPoses.get(pathPoses.size() - 1);
            pathPoses.remove(pathPoses.size() - 1);

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pathPoses);

            List<RotationTarget> rotationTargets = new ArrayList<>(pathPoses.size());

            // note we are starting from 1 to skip the first pose (the robot starting position)
            // ending at -2 to skip the final wall ride
            for (int i = 1; i < pathPoses.size() - 1; i++) {
                Pose2d p = pathPoses.get(i);
                rotationTargets.add(new RotationTarget(i, p.getRotation()));
            }

            // figure out what direction the wall ride is in (ccw or cw)
            z = r.fuelVision.findFieldZone(pathPoses.get(0));
            rect = r.fuelVision.fixedRects[z.ordinal()];
            mid = new Translation2d((rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2);
            Rotation2d angle1 =
                    pathPoses.get(pathPoses.size() - 1).getTranslation().minus(mid).getAngle();
            Rotation2d angle2 = nextCorner.getTranslation().minus(mid).getAngle();
            double result = angle2.minus(angle1).getRadians();
            double wallAngle = FuelVision.wallAngle;
            isCCW = false;
            if (result > 0) {
                isCCW = true;
                wallAngle = -wallAngle;
            }
            nextCorner =
                    new Pose2d(
                            nextCorner.getTranslation(),
                            nextCorner.getRotation().plus(Rotation2d.fromRadians(wallAngle)));

            if (angle2.getCos() > 0) {
                if (angle2.getSin() > 0) {
                    cornerIdx = 0;
                } else {
                    cornerIdx = 1;
                }
            } else {
                if (angle2.getSin() > 0) {
                    cornerIdx = 3;
                } else {
                    cornerIdx = 2;
                }
            }

            // offset the final angle by the wall angle
            rotationTargets.add(
                    new RotationTarget(
                            pathPoses.size() - 1,
                            pathPoses
                                    .get(pathPoses.size() - 1)
                                    .getRotation()
                                    .plus(Rotation2d.fromRadians(wallAngle))));

            PathPlannerPath path =
                    new PathPlannerPath(
                            waypoints,
                            rotationTargets,
                            List.of(),
                            List.of(),
                            List.of(),
                            globalConstraints,
                            // we do want to provide this so that we only generate one trajectory
                            // instead of 2
                            new IdealStartingState(
                                    Util2.getScalarVel(r.drive.getChassisSpeeds()),
                                    r.drive.getRotation()),
                            new GoalEndState(
                                    globalConstraints.maxVelocity(),
                                    rotationTargets.get(rotationTargets.size() - 1).rotation()),
                            false);
            path.preventFlipping = true;

            c =
                    AutoBuilder.followPath(path)
                            .andThen(driveToCorner().andThen(doCornerTurn()).repeatedly());
            c.initialize();

        } catch (Exception e) {
            System.out.println("Error generating fuel path:");
            e.printStackTrace();
        }
    }

    @Override
    public void execute() {
        // if(notFollowingPathWell()){
        //     c.end(true);
        //     regeneratePath();
        //     c.initialize();
        // }
        if (c != null) c.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (c != null) c.end(interrupted);
        r.drive.runVelocity(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        if (c != null) return c.isFinished();
        else return true;
    }

    private Command driveToCorner() {
        return new FunctionalCommand(
                () -> {}, this::driveToCornerExecute, (b) -> {}, this::driveToCornerIsFinished);
    }

    double kPx = 6;
    double kPy = 7;
    double kPangle = 3;
    double maxSpeed = 1.5;
    double controllerSpeed = 0.9;

    private void driveToCornerExecute() {
        double controllerX = -ConfigButtons.controller.getLeftY();
        double controllerY = -ConfigButtons.controller.getLeftX();

        // use the right controller axis according to robot movement
        switch (cornerIdx) {
            case 0:
            case 2:
                if (isCCW) {
                    controllerX = 0;
                } else {
                    controllerY = 0;
                }
                break;
            case 1:
            case 3:
            default:
                if (isCCW) {
                    controllerY = 0;
                } else {
                    controllerX = 0;
                }
                break;
        }

        // drive with a bit of PID and a bit of controller input
        Translation2d driveDirection =
                nextCorner.getTranslation().minus(r.drive.getPose().getTranslation());
        double xSpeed =
                MathUtil.clamp(driveDirection.getX() * kPx, -maxSpeed, maxSpeed)
                        + controllerX * controllerSpeed;
        double ySpeed =
                MathUtil.clamp(driveDirection.getY() * kPy, -maxSpeed, maxSpeed)
                        + controllerY * controllerSpeed;

        Rotation2d angleDiff = nextCorner.getRotation().minus(r.drive.getRotation());

        r.drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(xSpeed, ySpeed, angleDiff.getRadians() * kPangle),
                        r.drive.getRotation()));
    }

    private boolean driveToCornerIsFinished() {
        boolean isCloseEnough =
                r.drive.getPose().getTranslation().getDistance(nextCorner.getTranslation())
                        < Units.inchesToMeters(6);
        if (isCloseEnough) {
            // set next corner to the next corner
            // double[] {topleftx, topleftY, botRightx, botRightY}
            cornerIdx = (cornerIdx + (isCCW ? -1 : 1));
            if (cornerIdx > 3) cornerIdx = 0;
            else if (cornerIdx < 0) cornerIdx = 3;

            double wallAngle = nextCorner.getRotation().getRadians() - Math.PI / 2;
            if (isCCW) {
                wallAngle -= Math.PI;
            }
            switch (cornerIdx) {
                case 0:
                default:
                    nextCorner = new Pose2d(rect[0], rect[1], Rotation2d.fromRadians(wallAngle));
                    break;
                case 1:
                    nextCorner = new Pose2d(rect[0], rect[3], Rotation2d.fromRadians(wallAngle));
                    break;
                case 2:
                    nextCorner = new Pose2d(rect[2], rect[3], Rotation2d.fromRadians(wallAngle));
                    break;
                case 3:
                    nextCorner = new Pose2d(rect[2], rect[1], Rotation2d.fromRadians(wallAngle));
                    break;
            }
        }
        return isCloseEnough;
    }

    private Command doCornerTurn() {
        return new FunctionalCommand(
                () -> {}, this::doCornerTurnExecute, (b) -> {}, this::doCornerTurnIsFinished);
    }

    private double maxTurnXY = 0.4;
    private double angleTurnKp = 2;

    private void doCornerTurnExecute() {
        Translation2d driveDir = mid.minus(r.drive.getPose().getTranslation());
        driveDir = driveDir.times(1.0 / driveDir.getNorm());
        Rotation2d angleDelta = nextCorner.getRotation().minus(r.drive.getRotation());
        r.drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(
                                driveDir.getX() * maxTurnXY,
                                driveDir.getY() * maxTurnXY,
                                angleDelta.getRadians() * angleTurnKp),
                        r.drive.getRotation()));
    }

    private boolean doCornerTurnIsFinished() {
        return Math.abs(nextCorner.getRotation().minus(r.drive.getRotation()).getRadians())
                < Math.toRadians(20);
    }
}
