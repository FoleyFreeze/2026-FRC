package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Util {

    public static double floorMod(double a, double b) {
        // the google ai suggestion
        /*double r = a - b*Math.floor(a / b);
        if(r == b) return 0;
        if(b<0 && r>0 || b>0 && r<0){
            r += b;
        }
        return r;
        */

        // the we did it ourselves suggestion
        double r = a % b;
        if (r < 0 && b > 0 || r > 0 && b < 0) {
            r += b;
        }
        return r;
    }

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
    }
}
