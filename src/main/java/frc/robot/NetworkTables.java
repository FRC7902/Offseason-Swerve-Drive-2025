package frc.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTables {
    public static void main(String[] args){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        inst.startServer();

        NetworkTable visionTable = inst.getTable("Vision");

        visionTable.getEntry("MarkhamFirebirds_Camera").setString("CenterCamera");
        visionTable.getEntry("Distance").setDouble(12.5); // Example value, e.g., distance to target

        // Loop or listen for data (usually in a real robot program, you'd update in a loop)
        while (true) {
            // Here you can perform some logic or keep the server running
            // For example, send updated vision data periodically
            visionTable.getEntry("Angle").setDouble(45.0); // Example data
        }

    }
}
