package frc.robot.ShuffleHelper;


import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

public class Souffle {
    private final String prefix;
    // private static final Souffle root = new Souffle();

    Souffle() {
        prefix = "";
        // NetworkTableInstance.getDefault().getEntry("/Hey/Ho/Spagettio");
    }

    Souffle(String prefix) {
        this.prefix = prefix;
    }

    // public static <T> Object S() {
    // }

    // public <T> void set(String key, T value) {
    //     NetworkTableInstance.getDefault().getEntry(prefix+key).setValue(value);
    // }

    public static <T> void set(String key, T value) {
        NetworkTableInstance.getDefault().getEntry(key).setValue(value);
    }

    public static <T> void set(String tab, String name, T value) {
       set("/Shuffleboard/"+tab+"/"+name, value);
    }

    public static NetworkTableValue get(String tab, String name) {
        return NetworkTableInstance.getDefault().getEntry("/Shuffleboard/"+tab+"/"+name).getValue();
    }
    
    public static NetworkTableValue get(String key) {
        return NetworkTableInstance.getDefault().getEntry(key).getValue();
    }
}

