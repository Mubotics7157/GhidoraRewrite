package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim sim = new ElevatorSim(
            DCMotor.getFalcon500(2), // 2 Falcon 500 motors
            12.0, // Gear ratio
            5.0, // Carriage mass kg
            0.025, // Drum radius meters
            Constants.ElevatorConstants.MIN_HEIGHT_METERS,
            Constants.ElevatorConstants.MAX_HEIGHT_METERS,
            true, // Simulate gravity
            Constants.ElevatorConstants.MIN_HEIGHT_METERS // Starting position
            );

    private double appliedVolts = 0.0;
    private boolean closedLoop = false;
    private double positionSetpoint = 0.0;

    // Simple PID for Motion Magic simulation
    private final PIDController pid = new PIDController(
            Constants.ElevatorConstants.KP / 10.0, // Scale down for simulation
            Constants.ElevatorConstants.KI,
            Constants.ElevatorConstants.KD / 10.0);

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Run closed loop control if in position mode
        if (closedLoop) {
            double pidOutput = pid.calculate(sim.getPositionMeters(), positionSetpoint);
            appliedVolts = MathUtil.clamp(pidOutput, -12.0, 12.0);
        }

        sim.setInputVoltage(appliedVolts);
        sim.update(0.020); // 20ms update period

        inputs.positionMeters = sim.getPositionMeters();
        inputs.velocityMetersPerSecond = sim.getVelocityMetersPerSecond();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps() / 2.0, sim.getCurrentDrawAmps() / 2.0};
        inputs.tempCelsius = new double[] {20.0, 20.0}; // Assume room temperature

        // Convert to rotations for logging
        inputs.leftPositionRotations = sim.getPositionMeters() * 12.0; // Gear ratio
        inputs.leftVelocityRotationsPerSecond = sim.getVelocityMetersPerSecond() * 12.0;
        inputs.rightPositionRotations = inputs.leftPositionRotations;
        inputs.rightVelocityRotationsPerSecond = inputs.leftVelocityRotationsPerSecond;

        // Simulate limit switches
        inputs.topLimitSwitch = sim.getPositionMeters() >= Constants.ElevatorConstants.MAX_HEIGHT_METERS - 0.01;
        inputs.bottomLimitSwitch = sim.getPositionMeters() <= Constants.ElevatorConstants.MIN_HEIGHT_METERS + 0.01;
        inputs.carriageDetected = true; // Always detected in sim

        inputs.leftMotorConnected = true;
        inputs.rightMotorConnected = true;
    }

    @Override
    public void setVoltage(double volts) {
        closedLoop = false;
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setPosition(double positionMeters) {
        closedLoop = true;
        positionSetpoint = MathUtil.clamp(
                positionMeters,
                Constants.ElevatorConstants.MIN_HEIGHT_METERS,
                Constants.ElevatorConstants.MAX_HEIGHT_METERS);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        // No simulation needed for brake mode
    }

    @Override
    public void resetEncoder() {
        // Reset simulation position to minimum height
        sim = new ElevatorSim(
                DCMotor.getFalcon500(2),
                12.0,
                5.0,
                0.025,
                Constants.ElevatorConstants.MIN_HEIGHT_METERS,
                Constants.ElevatorConstants.MAX_HEIGHT_METERS,
                true,
                Constants.ElevatorConstants.MIN_HEIGHT_METERS);
    }
}
