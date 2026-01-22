package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;

public class ShooterInterp2d {
    public static record DataPoint(
        double rpm,
        double angle,
        double hood
    ){}

    //axes
    private static final double[] distAxis = {
        0,0,0,0,0,0,0,0,0,0,
    };
    private static final double[] velAxis = {
        0,0,0,0,0,
    };

    //tables
    private static final double[][] rpmTable =
    {
        //dist x
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0},
    //vel y
    };
    private static final double [][] turretAngleTable =
    {
        //dist x
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0},
    };
    private static final double [][] hoodAngleTable =
    {
        //dist x
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0},
    };

    public ShooterInterp2d(){

    }

    public DataPoint get(double dist){
        return get(dist,0);
    }
    public DataPoint get(double dist, double vel) {
        int distIdx = 1;
        int velIdx = 1;

        while (distIdx < distAxis.length - 1 && dist > distAxis[distIdx]){
            distIdx++;
        }
        while (velIdx < velAxis.length - 1 && vel > velAxis[velIdx]){
            velIdx++;
        }
        int pDistIdx = distIdx - 1;
        int pVelIdx = velIdx - 1;

        double fracX = MathUtil.inverseInterpolate(distAxis[pDistIdx], distAxis[distIdx], dist);
        double fracY = MathUtil.inverseInterpolate(velAxis[pVelIdx], velAxis[velIdx], vel);

        double interXpHI_rpm = MathUtil.interpolate(rpmTable[velIdx][pDistIdx], rpmTable[velIdx][distIdx], fracX);
        double interpXLo_rpm = MathUtil.interpolate(rpmTable[pVelIdx][pDistIdx], rpmTable[pVelIdx][distIdx], fracX);
        double interp_rpm = MathUtil.interpolate(interpXLo_rpm, interXpHI_rpm, fracY);

        double interXpHI_angle = MathUtil.interpolate(turretAngleTable[velIdx][pDistIdx], turretAngleTable[velIdx][distIdx], fracX);
        double interpXLo_angle = MathUtil.interpolate(turretAngleTable[pVelIdx][pDistIdx], turretAngleTable[pVelIdx][distIdx], fracX);
        double interp_angle = MathUtil.interpolate(interpXLo_angle, interXpHI_angle, fracY);
        
        double interXpHI_hood = MathUtil.interpolate(hoodAngleTable[velIdx][pDistIdx], hoodAngleTable[velIdx][distIdx], fracX);
        double interpXLo_hood = MathUtil.interpolate(hoodAngleTable[pVelIdx][pDistIdx], hoodAngleTable[pVelIdx][distIdx], fracX);
        double interp_hood = MathUtil.interpolate(interpXLo_hood, interXpHI_hood, fracY);

        return new DataPoint(interp_rpm, interp_angle, interp_hood);
    }
}
