package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private double goalPositionMeters = 0.0;
    private boolean closedLoop = false;

    // Mechanism2d visualization
    private final Mechanism2d mechanism;
    private final MechanismRoot2d elevatorRoot;
    private final MechanismLigament2d elevatorMast;
    private final MechanismLigament2d elevatorCarriage;
    private final MechanismLigament2d elevatorGoal;

    public Elevator(ElevatorIO io) {
        this.io = io;

        io.setBrakeMode(true);

        // Initialize Mechanism2d
        mechanism = new Mechanism2d(3.0, 2.5); // 3m wide, 2.5m tall display
        elevatorRoot = mechanism.getRoot("ElevatorBase", 1.5, 0.1); // Center horizontally, near bottom

        // Main elevator mast (vertical track)
        elevatorMast = elevatorRoot.append(new MechanismLigament2d(
                "Mast",
                Constants.ElevatorConstants.MAX_HEIGHT_METERS,
                90, // Vertical
                8, // Line width
                new Color8Bit(Color.kGray)));

        // Elevator carriage (moves up and down)
        elevatorCarriage = elevatorRoot.append(new MechanismLigament2d(
                "Carriage",
                0.2, // 20cm wide carriage
                0, // Horizontal
                12, // Line width
                new Color8Bit(Color.kBlue)));

        // Goal position indicator
        elevatorGoal = elevatorRoot.append(new MechanismLigament2d(
                "Goal",
                0.15, // 15cm wide goal indicator
                0, // Horizontal
                8, // Line width
                new Color8Bit(Color.kGreen)));

        // Add to SmartDashboard
        SmartDashboard.putData("Elevator Mechanism", mechanism);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // Safety checks
        if (inputs.topLimitSwitch && goalPositionMeters > inputs.positionMeters) {
            goalPositionMeters = inputs.positionMeters;
        }
        if (inputs.bottomLimitSwitch && goalPositionMeters < inputs.positionMeters) {
            goalPositionMeters = inputs.positionMeters;
        }

        // Run closed loop control
        if (closedLoop) {
            io.setPosition(goalPositionMeters);
        }

        // Update Mechanism2d visualization
        updateMechanism2d();

        // Log additional data
        Logger.recordOutput("Elevator/GoalPositionMeters", goalPositionMeters);
        Logger.recordOutput("Elevator/ClosedLoop", closedLoop);
        Logger.recordOutput("Elevator/AtSetpoint", atSetpoint());
        Logger.recordOutput("Elevator/ErrorMeters", goalPositionMeters - inputs.positionMeters);
    }

    private void updateMechanism2d() {
        // Update carriage position (height from base)
        double carriageHeight = Math.max(0.1, inputs.positionMeters + 0.1); // Add offset from base
        elevatorCarriage.setLength(carriageHeight);
        elevatorCarriage.setAngle(90); // Vertical position

        // Update goal position indicator
        double goalHeight = Math.max(0.1, goalPositionMeters + 0.1);
        elevatorGoal.setLength(goalHeight);
        elevatorGoal.setAngle(90);

        // Change colors based on state
        if (atSetpoint()) {
            elevatorCarriage.setColor(new Color8Bit(Color.kGreen));
        } else if (closedLoop) {
            elevatorCarriage.setColor(new Color8Bit(Color.kYellow));
        } else {
            elevatorCarriage.setColor(new Color8Bit(Color.kBlue));
        }

        // Show limit switch states
        if (inputs.topLimitSwitch) {
            elevatorMast.setColor(new Color8Bit(Color.kRed));
        } else if (inputs.bottomLimitSwitch) {
            elevatorMast.setColor(new Color8Bit(Color.kOrange));
        } else {
            elevatorMast.setColor(new Color8Bit(Color.kGray));
        }
    }

    /** Set the elevator to a specific height in meters using Motion Magic */
    public void setGoalPosition(double positionMeters) {
        goalPositionMeters = MathUtil.clamp(
                positionMeters,
                Constants.ElevatorConstants.MIN_HEIGHT_METERS,
                Constants.ElevatorConstants.MAX_HEIGHT_METERS);
        closedLoop = true;
    }

    /** Run the elevator at a specific voltage (open loop) */
    public void setVoltage(double volts) {
        closedLoop = false;

        // Safety checks for limit switches
        if (inputs.topLimitSwitch && volts > 0) {
            volts = 0;
        }
        if (inputs.bottomLimitSwitch && volts < 0) {
            volts = 0;
        }

        io.setVoltage(volts);
    }

    /** Stop the elevator */
    public void stop() {
        closedLoop = false;
        io.setVoltage(0.0);
    }

    /** Check if the elevator is at the setpoint */
    public boolean atSetpoint() {
        return closedLoop
                && Math.abs(goalPositionMeters - inputs.positionMeters)
                        < Constants.ElevatorConstants.POSITION_TOLERANCE_METERS;
    }

    /** Get the current elevator position in meters */
    public double getPosition() {
        return inputs.positionMeters;
    }

    /** Get the current elevator velocity in m/s */
    public double getVelocity() {
        return inputs.velocityMetersPerSecond;
    }

    /** Get the goal position in meters */
    public double getGoalPosition() {
        return goalPositionMeters;
    }

    /** Check if the top limit switch is triggered */
    public boolean isAtTop() {
        return inputs.topLimitSwitch;
    }

    /** Check if the bottom limit switch is triggered */
    public boolean isAtBottom() {
        return inputs.bottomLimitSwitch;
    }

    /** Check if carriage is detected */
    public boolean hasCarriage() {
        return inputs.carriageDetected;
    }

    /** Reset the elevator encoder to zero */
    public void resetEncoder() {
        io.resetEncoder();
    }

    /** Set brake mode */
    public void setBrakeMode(boolean enable) {
        io.setBrakeMode(enable);
    }

    /** Configure Motion Magic parameters */
    public void configureMotionMagic(double cruiseVelocityMPS, double accelerationMPS2, double jerkMPS3) {
        io.configureMotionMagic(cruiseVelocityMPS, accelerationMPS2, jerkMPS3);
    }
}
