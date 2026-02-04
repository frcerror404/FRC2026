// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final class FieldConstants {
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.2);
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(317.7);
    public static final double FIELD_CENTER_X_METERS = FIELD_LENGTH_METERS / 2.0;
    public static final double FIELD_CENTER_Y_METERS = FIELD_WIDTH_METERS / 2.0;
    public static final Translation2d FIELD_CENTER =
        new Translation2d(FIELD_CENTER_X_METERS, FIELD_CENTER_Y_METERS);
    public static final double BLUE_ALLIANCE_WALL_X_METERS = 0.0;
    public static final double RED_ALLIANCE_WALL_X_METERS = FIELD_LENGTH_METERS;
    public static final double CENTER_LINE_X_METERS = FIELD_CENTER_X_METERS;

    public static final double HUB_DISTANCE_FROM_ALLIANCE_WALL_METERS = Units.inchesToMeters(158.6);
    public static final double HUB_FACE_SIZE_METERS = Units.inchesToMeters(47.0);
    public static final double HUB_OPENING_FLAT_TO_FLAT_METERS = Units.inchesToMeters(41.7);
    public static final Translation2d BLUE_HUB_POSITION =
        new Translation2d(HUB_DISTANCE_FROM_ALLIANCE_WALL_METERS, FIELD_CENTER_Y_METERS);
    public static final Translation2d RED_HUB_POSITION =
        new Translation2d(
            FIELD_LENGTH_METERS - HUB_DISTANCE_FROM_ALLIANCE_WALL_METERS, FIELD_CENTER_Y_METERS);

    public static final double BUMP_WIDTH_METERS = Units.inchesToMeters(73.0);
    public static final double BUMP_DEPTH_METERS = Units.inchesToMeters(44.4);
    public static final double BUMP_HEIGHT_METERS = Units.inchesToMeters(6.513);
    public static final double BUMP_TOP_SURFACE_THICKNESS_METERS = Units.inchesToMeters(0.5);

    public static final double TRENCH_WIDTH_METERS = Units.inchesToMeters(65.65);
    public static final double TRENCH_DEPTH_METERS = Units.inchesToMeters(47.0);
    public static final double TRENCH_HEIGHT_METERS = Units.inchesToMeters(40.25);
    public static final double TRENCH_CLEAR_OPENING_WIDTH_METERS = Units.inchesToMeters(50.34);
    public static final double TRENCH_CLEAR_OPENING_HEIGHT_METERS = Units.inchesToMeters(22.25);

    public static final double DEPOT_WIDTH_METERS = Units.inchesToMeters(42.0);
    public static final double DEPOT_DEPTH_METERS = Units.inchesToMeters(27.0);
    public static final double DEPOT_BARRIER_WIDTH_METERS = Units.inchesToMeters(3.0);
    public static final double DEPOT_BARRIER_HEIGHT_METERS = Units.inchesToMeters(1.0);
    public static final double DEPOT_BARRIER_HEIGHT_WITH_FASTENER_METERS =
        Units.inchesToMeters(1.125);

    public static final double TOWER_WIDTH_METERS = Units.inchesToMeters(49.25);
    public static final double TOWER_DEPTH_METERS = Units.inchesToMeters(45.0);
    public static final double TOWER_HEIGHT_METERS = Units.inchesToMeters(78.25);
    public static final double TOWER_BASE_WIDTH_METERS = Units.inchesToMeters(39.0);
    public static final double TOWER_BASE_DEPTH_METERS = Units.inchesToMeters(45.18);
    public static final double TOWER_BASE_EDGE_MIN_HEIGHT_METERS = Units.inchesToMeters(0.2);
    public static final double TOWER_BASE_EDGE_MAX_HEIGHT_METERS = Units.inchesToMeters(0.3);
    public static final double TOWER_UPRIGHT_SPACING_METERS = Units.inchesToMeters(32.25);

    public static final double TOWER_LOW_RUNG_HEIGHT_METERS = Units.inchesToMeters(27.0);
    public static final double TOWER_MID_RUNG_HEIGHT_METERS = Units.inchesToMeters(45.0);
    public static final double TOWER_HIGH_RUNG_HEIGHT_METERS = Units.inchesToMeters(63.0);
    public static final double TOWER_RUNG_SPACING_METERS = Units.inchesToMeters(18.0);
    public static final double TOWER_RUNG_OFFSET_FROM_UPRIGHT_METERS = Units.inchesToMeters(5.875);

    public static final double APRILTAG_SIZE_METERS = Units.inchesToMeters(8.125);
    public static final double TOWER_WALL_TAG_CENTER_HEIGHT_METERS = Units.inchesToMeters(21.75);
    public static final double TRENCH_TAG_CENTER_HEIGHT_METERS = Units.inchesToMeters(35.0);

    private FieldConstants() {}

    public static Translation2d getHubPosition(Alliance alliance) {
      return alliance == Alliance.Red ? RED_HUB_POSITION : BLUE_HUB_POSITION;
    }
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
