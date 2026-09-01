package frc.robot.subsystems;

import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NTelemetryDashboard {
    private NetworkTableInstance NT = NetworkTableInstance.getDefault();
    public NetworkTable telemetry = NT.getTable("telemetry");


    public NTelemetryDashboard() { };

    public void putNumber(String subtableName, String name, double value) {        
        telemetry.getSubTable(subtableName).getEntry(name).setValue(NetworkTableValue.makeDouble(value));
    };

    public void putString(String subtableName, String name, String value) {        
        telemetry.getSubTable(subtableName).getEntry(name).setValue(NetworkTableValue.makeString(value));
    };

    public void putBoolean(String subtableName, String name, Boolean value) {        
        telemetry.getSubTable(subtableName).getEntry(name).setValue(NetworkTableValue.makeBoolean(value));
    };

    public void putColor(String subtableName, String name, int r, int g, int b) {        
        String color = String.format("#%02X%02X%02X", r, g, b);

        telemetry.getSubTable(subtableName).getEntry(name).setValue(NetworkTableValue.makeString(color));
    };

    // percent is from 0 - 1 as a percentage going from rgb1 - rgb2
    public void putColorGradient(String subtableName, String name, int percent, int r1, int g1, int b1, int r2, int g2, int b2) {
        int r = ((percent * 100 * r1) + ((1 - percent) * 100 * r2)) / 100;
        int g = ((percent * 100 * g1) + ((1 - percent) * 100 * g2)) / 100;
        int b = ((percent * 100 * b1) + ((1 - percent) * 100 * b2)) / 100;

        String color = String.format("#%02X%02X%02X", r, g, b);

        telemetry.getSubTable(subtableName).getEntry(name).setValue(NetworkTableValue.makeString(color));
    };
};