package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSecond = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};

        public boolean topLimitSwitch = false;
        public boolean bottomLimitSwitch = false;
        public boolean carriageDetected = false;

        public double leftPositionRotations = 0.0;
        public double leftVelocityRotationsPerSecond = 0.0;
        public double rightPositionRotations = 0.0;
        public double rightVelocityRotationsPerSecond = 0.0;

        public boolean leftMotorConnected = false;
        public boolean rightMotorConnected = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Run the elevator motors at the specified voltage. */
    public default void setVoltage(double volts) {}

    /** Set the elevator position using Motion Magic. */
    public default void setPosition(double positionMeters) {}

    /** Set the elevator motor brake mode. */
    public default void setBrakeMode(boolean enable) {}

    /** Reset the elevator encoder position to zero. */
    public default void resetEncoder() {}

    /** Configure the Motion Magic parameters. */
    public default void configureMotionMagic(double cruiseVelocity, double acceleration, double jerk) {}
}
