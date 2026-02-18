// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.RobotContainer;

// public class ShooterSequential extends SequentialCommandGroup {
//     private static final CommandXboxController controller = new CommandXboxController(0);

//     public ShooterSequential(RobotContainer r, CommandXboxController controller) {
//         addCommands(
//                 new RobotContainer()
//                         .shooter.cameraShoot(r.drive::getPose, r.drive::getFieldVelocity),
//                 new RobotContainer()
//                         .spindexter
//                         .smartSpinCmd(r.shooter, r.drive)
//                         .alongWith(DriveCommands.slowDrive(controller, r.drive)));
//     }
// }
