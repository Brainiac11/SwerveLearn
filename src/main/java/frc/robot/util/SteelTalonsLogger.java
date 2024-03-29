package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SteelTalonsLogger {
    private static ShuffleboardTab mainTab;
    private static HashMap<String, GenericEntry> widgetList;
    public SteelTalonsLogger() {
        mainTab = Shuffleboard.getTab("5427_Logger");
        widgetList = new HashMap<String, GenericEntry>();
    }

    public static boolean post(String key, Object obj) {
        if (!widgetList.containsKey(key)) {
            widgetList.put(key, mainTab.add(key, obj).getEntry());
            return widgetList.get(key).setValue(obj);
        } else {
            return widgetList.get(key).setValue(obj);
        }
    }

    public static void postComplex(String key, Sendable sendable) {
        try {
            mainTab.add(key, sendable);
        } catch (Exception e) {
            return;
        }
    }
}