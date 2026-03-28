// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  @AutoLogOutput(key = "RobotMode")
  public static Mode getMode() {
    if (RobotBase.isReal()) {
      return Mode.REAL;
    } else {
      return Mode.SIM;
    }
  }

  public static final CANBus CANIVORE_BUS = new CANBus("canivore");
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }
}
