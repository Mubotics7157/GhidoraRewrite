// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
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

    public static final class ElevatorConstants {
        // CAN IDs
        public static final int LEFT_MOTOR_ID = 10;
        public static final int RIGHT_MOTOR_ID = 11;

        // DIO ports
        public static final int TOP_LIMIT_DIO = 0;
        public static final int BOTTOM_LIMIT_DIO = 1;
        public static final int CARRIAGE_SENSOR_DIO = 2;

        // Physical constraints (meters)
        public static final double MIN_HEIGHT_METERS = 0.0;
        public static final double MAX_HEIGHT_METERS = 1.5;
        public static final double POSITION_TOLERANCE_METERS = 0.02;

        // PID constants (tune these values using Phoenix Tuner X)
        public static final double KP = 24.0; // Proportional gain
        public static final double KI = 0.0; // Integral gain
        public static final double KD = 0.1; // Derivative gain

        // Feedforward constants (characterize using SysId)
        public static final double KS = 0.25; // Static friction (volts)
        public static final double KV = 2.4; // Velocity constant (volts per rotation/sec)
        public static final double KA = 0.05; // Acceleration constant (volts per rotation/sec²)

        // Motion Magic constants (in rotations and rotations/sec)
        public static final double CRUISE_VELOCITY_ROTATIONS_PER_SEC = 8.0; // ~0.67 m/s
        public static final double MAX_ACCELERATION_ROTATIONS_PER_SEC_SQUARED = 16.0; // ~1.33 m/s²
        public static final double MAX_JERK_ROTATIONS_PER_SEC_CUBED = 160.0; // ~13.3 m/s³

        // Preset positions
        public static final double BOTTOM_POSITION = 0.0;
        public static final double LOW_POSITION = 0.3;
        public static final double MID_POSITION = 0.75;
        public static final double HIGH_POSITION = 1.2;
        public static final double TOP_POSITION = 1.4;
    }
}
