package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class GetAlliance {
    public static boolean isRed() {
        switch (DriverStation.getRawAllianceStation()) {
            case Red1: case Red2: case Red3: return true;
            case Blue1: case Blue2: case Blue3: return false;
            default: return true; // fix
        }
    }
}
