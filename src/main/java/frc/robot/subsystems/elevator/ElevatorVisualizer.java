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

package frc.robot.subsystems.elevator;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

/**
 * Mechanism2d visualizer for the elevator subsystem. Reads data from AdvantageKit logging and displays real-time
 * elevator state.
 */
public class ElevatorVisualizer {
    private final Mechanism2d mechanism;
    private final MechanismRoot2d elevatorRoot;
    private final MechanismLigament2d elevatorMast;
    private final MechanismLigament2d elevatorCarriage;
    private final MechanismLigament2d elevatorGoal;

    // NetworkTables for reading AdvantageKit logged data
    private final NetworkTable elevatorTable;

    // Constants for visualization
    private static final double MECHANISM_WIDTH = 3.0; // meters
    private static final double MECHANISM_HEIGHT = 2.5; // meters
    private static final double BASE_OFFSET = 0.1; // meters from bottom
    private static final double CARRIAGE_WIDTH = 0.2; // meters
    private static final double GOAL_WIDTH = 0.15; // meters

    public ElevatorVisualizer() {
        // Initialize Mechanism2d canvas
        mechanism = new Mechanism2d(MECHANISM_WIDTH, MECHANISM_HEIGHT);
        elevatorRoot = mechanism.getRoot("ElevatorBase", MECHANISM_WIDTH / 2.0, BASE_OFFSET);

        // Create elevator mast (vertical track)
        elevatorMast = elevatorRoot.append(new MechanismLigament2d(
                "Mast",
                Constants.ElevatorConstants.MAX_HEIGHT_METERS,
                90, // Vertical (90 degrees)
                8, // Line width
                new Color8Bit(Color.kGray)));

        // Create elevator carriage (moves up and down)
        elevatorCarriage = elevatorRoot.append(new MechanismLigament2d(
                "Carriage",
                CARRIAGE_WIDTH,
                0, // Horizontal initially
                12, // Line width (thicker than mast)
                new Color8Bit(Color.kBlue)));

        // Create goal position indicator
        elevatorGoal = elevatorRoot.append(new MechanismLigament2d(
                "Goal",
                GOAL_WIDTH,
                0, // Horizontal initially
                8, // Line width
                new Color8Bit(Color.kGreen)));

        // Initialize NetworkTables connection to AdvantageKit logged data
        // This reads from the published AdvantageKit outputs
        elevatorTable = NetworkTableInstance.getDefault().getTable("AdvantageKit/RealOutputs/Elevator");

        // Publish mechanism to SmartDashboard for viewing
        SmartDashboard.putData("Elevator Mechanism", mechanism);
    }

    /**
     * Updates the mechanism visualization with current elevator state. Call this periodically (typically from
     * Robot.simulationPeriodic()).
     */
    public void update() {
        // Read current values from AdvantageKit NetworkTables
        double currentPosition = elevatorTable.getEntry("PositionMeters").getDouble(0.0);
        double goalPosition = elevatorTable.getEntry("GoalPositionMeters").getDouble(0.0);
        boolean atSetpoint = elevatorTable.getEntry("AtSetpoint").getBoolean(false);
        boolean closedLoop = elevatorTable.getEntry("ClosedLoop").getBoolean(false);
        boolean topLimit = elevatorTable.getEntry("TopLimitSwitch").getBoolean(false);
        boolean bottomLimit = elevatorTable.getEntry("BottomLimitSwitch").getBoolean(false);

        // Update the visual elements
        updateMechanism2d(currentPosition, goalPosition, atSetpoint, closedLoop, topLimit, bottomLimit);
    }

    /** Updates the Mechanism2d visual elements based on elevator state. */
    private void updateMechanism2d(
            double currentPosition,
            double goalPosition,
            boolean atSetpoint,
            boolean closedLoop,
            boolean topLimit,
            boolean bottomLimit) {

        // Update carriage position (height from base)
        // Add small offset to keep carriage visible above base
        double carriageHeight = Math.max(BASE_OFFSET, currentPosition + BASE_OFFSET);
        elevatorCarriage.setLength(carriageHeight);
        elevatorCarriage.setAngle(90); // Vertical position to show height

        // Update goal position indicator
        double goalHeight = Math.max(BASE_OFFSET, goalPosition + BASE_OFFSET);
        elevatorGoal.setLength(goalHeight);
        elevatorGoal.setAngle(90); // Vertical position to show height

        // Update carriage color based on elevator state
        if (atSetpoint) {
            // Green when at setpoint
            elevatorCarriage.setColor(new Color8Bit(Color.kGreen));
        } else if (closedLoop) {
            // Yellow when moving to position
            elevatorCarriage.setColor(new Color8Bit(Color.kYellow));
        } else {
            // Blue for manual control or stopped
            elevatorCarriage.setColor(new Color8Bit(Color.kBlue));
        }

        // Update mast color based on limit switch states
        if (topLimit) {
            // Red when top limit is hit
            elevatorMast.setColor(new Color8Bit(Color.kRed));
        } else if (bottomLimit) {
            // Orange when bottom limit is hit
            elevatorMast.setColor(new Color8Bit(Color.kOrange));
        } else {
            // Gray for normal operation
            elevatorMast.setColor(new Color8Bit(Color.kGray));
        }

        // Make goal indicator more visible when active
        if (closedLoop) {
            elevatorGoal.setColor(new Color8Bit(Color.kLime)); // Bright green when actively seeking
        } else {
            elevatorGoal.setColor(new Color8Bit(Color.kGreen)); // Darker green when not active
        }
    }

    /**
     * Gets the Mechanism2d object for custom dashboard integration if needed.
     *
     * @return The mechanism visualization object
     */
    public Mechanism2d getMechanism() {
        return mechanism;
    }
}
