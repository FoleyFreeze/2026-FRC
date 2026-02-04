package frc.robot.subsystems.stats;

import edu.wpi.first.util.struct.Struct;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.nio.ByteBuffer;

public class StatsStructReflection implements Struct<Stats> {
    // try to implement Struct by abusing reflection
    // warning: dont try this at home, etc etc

    public static int sizeBytes;
    public static int sizeObjects;
    public static String schema;
    public static Field[] activeFields;
    public static Constructor<?> activeConstructor;

    static {
        sizeBytes = 0;
        sizeObjects = 0;
        schema = "";

        // count the number of double fields
        Field[] fields = Stats.class.getDeclaredFields();
        for (Field f : fields) {
            if (f.getType().equals(Double.TYPE)) {
                sizeBytes += kSizeDouble;
                sizeObjects++;
            }
        }

        // loop through fields again to save the ones we care about
        // also build the schema string
        activeFields = new Field[sizeObjects];
        int j = 0;
        for (int i = 0; i < fields.length; i++) {
            Field f = fields[i];
            if (f.getType().equals(Double.TYPE)) {
                activeFields[j++] = f;

                if (!schema.isBlank()) schema += ";";
                schema += "double " + f.getName();
            }
        }

        // loop through constructors and grab the big one
        Constructor<?>[] cons = Stats.class.getConstructors();
        for (Constructor<?> c : cons) {
            if (c.getParameterCount() == sizeObjects) {
                activeConstructor = c;
            }
        }
    }

    @Override
    public Class<Stats> getTypeClass() {
        return Stats.class;
    }

    @Override
    public String getTypeName() {
        return "Stats";
    }

    @Override
    public int getSize() {
        return sizeBytes;
    }

    @Override
    public String getSchema() {
        return schema;
    }

    @Override
    public Stats unpack(ByteBuffer bb) {
        final Stats defaultReturn = new Stats();
        final Object[] o = new Object[sizeObjects];

        // construct a list of inputs
        for (int i = 0; i < o.length; i++) {
            o[i] = bb.getDouble();
        }

        Stats s;
        try {
            // apply that to the saved constructor
            s = (Stats) activeConstructor.newInstance(o);
        } catch (Exception e) {
            e.printStackTrace();
            s = defaultReturn;
        }

        return s;
    }

    @Override
    public void pack(ByteBuffer bb, Stats value) {
        try {
            // for each preselected field, put its value in the byte buffer
            for (Field f : activeFields) {
                bb.putDouble((Double) f.get(value));
            }
        } catch (Exception e) {
            e.printStackTrace();
            // at least pack something if it failed
            for (int i = 0; i < sizeObjects; i++) {
                bb.putDouble(0);
            }
        }
    }

    @Override
    public boolean isImmutable() {
        return false;
    }
}
