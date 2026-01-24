package frc.robot.util;

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
}
