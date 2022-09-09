package frc.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// right now, this just wraps the SmartDashboard class
// but using this will allow easier migration to superior
// systems in the future, either other implementations
// of the network tables api or an entirely independant
// system

public class Telemetry {
    public static void setValue ( String key, double value ) {
        SmartDashboard.putNumber(key, value);
    }

    public static void setValue ( String key, String value ) {
        SmartDashboard.putString(key, value);
    }

    public static void setValue ( String key, Boolean value ) {
        SmartDashboard.putBoolean(key, value);
    }

    public static double getValue ( String key, Double defaultValue ) {
        return SmartDashboard.getNumber(key, defaultValue);
    }

    public static String getValue ( String key, String defaultValue ) {
        return SmartDashboard.getString(key, defaultValue);
    }

    public static Boolean getValue ( String key, Boolean defaultValue ) {
        return SmartDashboard.getBoolean(key, defaultValue);
    }
}
