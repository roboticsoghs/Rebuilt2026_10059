package frc.robot.subsystems;

import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NTelemetryDashboard {
    private NetworkTableInstance NT = NetworkTableInstance.getDefault();
    public NetworkTable telemetry = NT.getTable("telemetry");

    public List<NetworkTable> subtables = new ArrayList<>();

    NTelemetryDashboard() { };

    public void newSubtable(String name) {
        subtables.add(telemetry.getSubTable(name));
    }

    public void putNumber(String name, double value) {
        
    };
};