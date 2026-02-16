// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class TurretConstants {
    public static double launchHeight = 0.8; // m
    public static double exitVelocity = 8.0; // m/s
    public static double aimTolerance = 0.50; // m
    public static final int LEFT_MOTOR = 50;
    public static final int RIGHT_MOTOR = 51;
    public static final int HOOD_MOTOR = 33;
    public static final int TURN_MOTOR = 60;
    public static final int FEEDER_MOTOR = 40;
    public static final int INDEX_MOTOR = 52;
  }

  public static class FieldConstants {
    public static double blueHubX = 4.62;
    public static double blueHubY = 4.03;
    public static Pair<Double, Double> blueHubLoc = new Pair<>(blueHubX, blueHubY);

    public static double redHubX = 0.00;
    public static double redHubY = 4.03;
    public static Pair<Double, Double> redHubLoc = new Pair<>(redHubX, redHubY);

    public static double hubHeight = 1.82; // m
  }

  // You should probably find a better place to put this?
  public static class UniverseConstants {
    public static double g = 9.81;
    public static double redhubX = Units.inchesToMeters(651.22 - 182.11);
    public static double redhubY = Units.inchesToMeters(158.84);
    public static double bluehubX = Units.inchesToMeters(182.11);
    public static double bluehubY = Units.inchesToMeters(158.84);
  }

  public static class ClimberConstants {
    public static final int MAX_AMP_POWER = 60;
    public static final double MAX_VOLTAGE = 12;
    public static final double KP = 0.5;
    public static final double MAX_MOTOR_OUTPUT = 1;
  }
}
