package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

/**
 * Collection of utilities for Lynx modules.
 */
public class LynxModuleUtil {
    private static final int MIN_VERSION = 1;
    private static final int MIN_REVISION = 8;
    private static final int MIN_BUILD = 2;

    /**
     * Ensure all of the Lynx modules attached to the robot satisfy the minimum requirement.
     * @param hardwareMap hardware map containing Lynx modules
     */
    public static void ensureMinimumFirmwareVersion(HardwareMap hardwareMap) {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            LynxModule.FirmwareVersion version = module.getFirmwareVersion();

            if (version.equals(LynxModule.FirmwareVersion.UNKNOWN_VERSION)) {
                throw new RuntimeException(Misc.formatInvariant(
                        "The firmware version of the Control Hub or Expansion Hub at \"%s\" is too old.\n" +
                        "Expected version %d.%d.%d or higher.\n" +
                        "Actual version is UNKNOWN_VERSION.\n" +
                        "The firmware version can be found in the Control Hub or Expansion Hub web interface.\n" +
                        "Contact info@revrobotics.com if you need help updating your firmware.",
                        module.getConnectionInfo(), MIN_VERSION, MIN_REVISION, MIN_BUILD));
            }

            if (!version.isAtLeast(MIN_VERSION, MIN_REVISION, MIN_BUILD)) {
                throw new RuntimeException(Misc.formatInvariant(
                        "The firmware version of the Control Hub or Expansion Hub at \"%s\" is too old.\n" +
                        "Expected version %d.%d.%d or higher.\n" +
                        "Actual version is %s.\n" +
                        "The firmware version can be found in the Control Hub or Expansion Hub web interface.\n" +
                        "Contact info@revrobotics.com if you need help updating your firmware.",
                        module.getConnectionInfo(), MIN_VERSION, MIN_REVISION, MIN_BUILD, version));
            }
        }
    }
}