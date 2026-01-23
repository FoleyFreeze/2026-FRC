package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShooterInterp1d.DataPoint;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final ShooterInterp1d lerp = new ShooterInterp1d();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public Command prime() {
        return new RunCommand(() -> io.wheelPower(1), this);
    }

    public Command stop() {
        return new RunCommand(() -> io.wheelPower(0), this);
    }

    public void goalPrime(Pose2d botLoc, ChassisSpeeds botVel) {
        // 1 collect the data to call the lerp
        Translation2d goal = FieldConstants.flipIfRed(FieldConstants.Hub.center);
        Translation2d bot =
                botLoc.getTranslation()
                        .plus(Constants.shooterLocOnBot.rotateBy(botLoc.getRotation()));

        // 2 call the lerp
        DataPoint setpoints = lerp.get(goal, bot, botVel);

        // 3 use setpoints from lerp to set motors
        io.setAll(setpoints.angle() - botLoc.getRotation().getDegrees(), 0, 0);
    }
}
