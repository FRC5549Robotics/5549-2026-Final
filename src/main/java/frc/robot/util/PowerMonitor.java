package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class PowerMonitor {
    private final PowerDistribution pdh;

    private final DoubleLogEntry totalCurrentLog;
    private final DoubleLogEntry voltageLog;

    private final DoubleLogEntry[] channelLogs;

    public PowerMonitor(String[] channelNames) {
        pdh = new PowerDistribution(1, ModuleType.kRev); //check that PDH can ID is 1

        DataLog log = DataLogManager.getLog();

        totalCurrentLog = new DoubleLogEntry(log, "/pdh/total_current");
        voltageLog = new DoubleLogEntry(log, "/pdh/voltage");

        channelLogs = new DoubleLogEntry[channelNames.length];
        for (int i = 1; i < channelNames.length; i++) {
            if (channelNames[i] != null) {
                channelLogs[i] = new DoubleLogEntry(log, "/pdh/channels/" + channelNames[i]);
            }
        }
    }

    public void log() {
        totalCurrentLog.append(pdh.getTotalCurrent());
        voltageLog.append(pdh.getVoltage());

        for (int i=1; i < channelLogs.length; i++) {
            if (channelLogs[i] != null) {
                channelLogs[i].append(pdh.getCurrent(i));
            }
        }
    }

}
