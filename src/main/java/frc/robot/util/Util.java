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

    public static double wrapAngle(double currentAngle, double targetModulo) {
        double delta = currentAngle % targetModulo;
        double shortDelta;
        if (delta > targetModulo / 2) {
            shortDelta = delta - targetModulo;
        } else if (delta < -targetModulo / 2) {
            shortDelta = delta + targetModulo;
        } else {
            shortDelta = delta;
        }
        double setPoint = currentAngle - shortDelta;
        return setPoint;
    }
}
