package frc.robot.ShuffleHelper;

import edu.wpi.first.networktables.NetworkTableInstance;

public class ShuffleUtil {
    

    ShuffleUtil() {
    }

    public static <T> void set(String tab, String name, T value) {
        NetworkTableInstance.getDefault().getEntry("/Shuffleboard/"+tab+"/"+name).setValue(value);
        // @Test
        // int x = 1;

        // Shuffleboard.getTab(tab);
        // SmartDashboard.getEntry(tab)
        // SmartDashboard.putBoolean(tab, false)
    }

    public static Object get(String tab, String name) {
        return NetworkTableInstance.getDefault().getEntry("/"+tab+"/"+name).getValue();
    }
}
