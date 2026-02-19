package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    RobotContainer r;
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    public static final double climberHighEnough = 30;
    public static final double robotHighEnough = 15; //TODO: add real vals 
    public static final double climberReleaseThresh = 40;
    public static final double climberBackIn = 0;
    public static final double extendPower = 0.5;
    public static final double retractPower = -0.5;
    public static final double liftPower = -1;

    public Climber(ClimberIO io, RobotContainer r) {
        this.io = io;
        this.r = r;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        Logger.recordOutput("Climber/Pos", inputs.climb1Position);
        Logger.recordOutput("Climber/Voltage", inputs.climb1Voltage);
    }

    public double getClimbPos1() {
        return inputs.climb1Position;
    }

    public double getClimbPos2() {
        return inputs.climb2Position;
    }

    public Command climbStop(){
        return new InstantCommand(()-> io.setPower1(0), this);
    }
    //this sucks so bad, if we have time or want to climb better, rewrite to be a single thingy
    


    public Command autoExtendCmd() { // step 1; goes up and grabs the bar
        return new RunCommand(() -> io.setPower1(extendPower), this).until(()-> inputs.climb1Position > climberHighEnough).andThen(climbStop());
    }

    public Command liftUpBotCmd(){ // step 2: lifts the bot to do the acutal climb part
        return new RunCommand(()-> io.setPower1(liftPower), this).until(()-> inputs.climb1Position < robotHighEnough).andThen(climbStop());
    }
    //start of go down
    public Command pushDownBotCmd(){ // step 3: after the climb, it pushes the bot down and unclimbs (pushes up to release from the bar)
        return new RunCommand(()-> io.setPower1(extendPower), this).until(()-> inputs.climb1Position > climberReleaseThresh).andThen(climbStop());
    }
    public Command retractClimberCmd(){  // step 4: returns climber to its original space of being retracted
        return new RunCommand(()-> io.setPower1(retractPower), this).until(()-> inputs.climb1Position < climberBackIn).andThen(climbStop());
    }
}
