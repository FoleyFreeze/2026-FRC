package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class WaitUntilNoBalls extends Command{
    double initTime,emptyTime;
    double lastBallCount;
    private Timer initTimer;
    private Timer emptyTimer;

    public WaitUntilNoBalls(double initTime, double emptyTime){
        this.initTime = initTime;
        this.emptyTime = emptyTime;
        initTimer = new Timer();
        emptyTimer = new Timer();
    }

    @Override
    public void initialize(){
        initTimer.restart();
        emptyTimer.restart();
        lastBallCount = RobotContainer.getInstance().stats.stats.totalBallShots;
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        if()
    }

    @Override
    public void end(boolean interrupted){

    }
}
