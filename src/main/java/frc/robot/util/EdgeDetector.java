package frc.robot.util;

public class EdgeDetector {
    public enum EdgeType {
        RISING,
        FALLING,
        BOTH
    }

    private EdgeType type;
    private boolean prevValue = false;

    public EdgeDetector(EdgeType type) {
        this.type = type;
    }

    public boolean calc(boolean in) {
        boolean out;
        switch (type) {
            case RISING:
                out = in && !prevValue;
                break;

            case FALLING:
                out = !in && prevValue;
                break;

            case BOTH:
            default:
                out = in ^ prevValue;
                break;
        }
        prevValue = in;
        return out;
    }
}
