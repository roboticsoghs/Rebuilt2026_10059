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
};