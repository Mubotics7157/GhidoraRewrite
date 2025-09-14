package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private final DigitalInput topLimit;
    private final DigitalInput bottomLimit;
    private final DigitalInput carriageSensor;

    // Status signals
    private final StatusSignal<Angle> leftPosition;
    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<Voltage> leftAppliedVolts;
    private final StatusSignal<Current> leftCurrent;
    private final StatusSignal<Temperature> leftTemp;

    private final StatusSignal<Angle> rightPosition;
    private final StatusSignal<AngularVelocity> rightVelocity;
    private final StatusSignal<Voltage> rightAppliedVolts;
    private final StatusSignal<Current> rightCurrent;
    private final StatusSignal<Temperature> rightTemp;

    // Control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    // Conversion factors
    private static final double GEAR_RATIO = 12.0; // Motor rotations per elevator meter
    private static final double ROTATIONS_TO_METERS = 1.0 / GEAR_RATIO;
    private static final double METERS_TO_ROTATIONS = GEAR_RATIO;

    public ElevatorIOTalonFX() {
        // Initialize motors
        leftMotor = new TalonFX(Constants.ElevatorConstants.LEFT_MOTOR_ID);
        rightMotor = new TalonFX(Constants.ElevatorConstants.RIGHT_MOTOR_ID);

        // Configure motors
        var leftConfig = new TalonFXConfiguration();
        var rightConfig = new TalonFXConfiguration();

        // Motor output settings
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Current limits
        leftConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Voltage compensation
        leftConfig.Voltage.PeakForwardVoltage = 12.0;
        leftConfig.Voltage.PeakReverseVoltage = -12.0;
        rightConfig.Voltage.PeakForwardVoltage = 12.0;
        rightConfig.Voltage.PeakReverseVoltage = -12.0;

        // PID Configuration
        var slot0 = new Slot0Configs();
        slot0.kP = Constants.ElevatorConstants.KP;
        slot0.kI = Constants.ElevatorConstants.KI;
        slot0.kD = Constants.ElevatorConstants.KD;
        slot0.kS = Constants.ElevatorConstants.KS;
        slot0.kV = Constants.ElevatorConstants.KV;
        slot0.kA = Constants.ElevatorConstants.KA;

        leftConfig.Slot0 = slot0;
        rightConfig.Slot0 = slot0;

        // Motion Magic Configuration
        var motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = Constants.ElevatorConstants.CRUISE_VELOCITY_ROTATIONS_PER_SEC;
        motionMagicConfig.MotionMagicAcceleration =
                Constants.ElevatorConstants.MAX_ACCELERATION_ROTATIONS_PER_SEC_SQUARED;
        motionMagicConfig.MotionMagicJerk = Constants.ElevatorConstants.MAX_JERK_ROTATIONS_PER_SEC_CUBED;

        leftConfig.MotionMagic = motionMagicConfig;
        rightConfig.MotionMagic = motionMagicConfig;

        // Apply configurations
        leftMotor.getConfigurator().apply(leftConfig);
        rightMotor.getConfigurator().apply(rightConfig);

        // Set right motor to follow left motor
        rightMotor.setControl(new com.ctre.phoenix6.controls.Follower(Constants.ElevatorConstants.LEFT_MOTOR_ID, true));

        // Initialize limit switches and sensors
        topLimit = new DigitalInput(Constants.ElevatorConstants.TOP_LIMIT_DIO);
        bottomLimit = new DigitalInput(Constants.ElevatorConstants.BOTTOM_LIMIT_DIO);
        carriageSensor = new DigitalInput(Constants.ElevatorConstants.CARRIAGE_SENSOR_DIO);

        // Initialize status signals
        leftPosition = leftMotor.getPosition();
        leftVelocity = leftMotor.getVelocity();
        leftAppliedVolts = leftMotor.getMotorVoltage();
        leftCurrent = leftMotor.getSupplyCurrent();
        leftTemp = leftMotor.getDeviceTemp();

        rightPosition = rightMotor.getPosition();
        rightVelocity = rightMotor.getVelocity();
        rightAppliedVolts = rightMotor.getMotorVoltage();
        rightCurrent = rightMotor.getSupplyCurrent();
        rightTemp = rightMotor.getDeviceTemp();

        // Set update frequencies (100Hz for important signals, 50Hz for others)
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, leftPosition, leftVelocity, rightPosition, rightVelocity);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, leftAppliedVolts, leftCurrent, leftTemp, rightAppliedVolts, rightCurrent, rightTemp);

        // Optimize bus utilization
        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Refresh all signals
        BaseStatusSignal.refreshAll(
                leftPosition,
                leftVelocity,
                leftAppliedVolts,
                leftCurrent,
                leftTemp,
                rightPosition,
                rightVelocity,
                rightAppliedVolts,
                rightCurrent,
                rightTemp);

        // Convert rotations to meters for main position/velocity
        inputs.positionMeters = leftPosition.getValueAsDouble() * ROTATIONS_TO_METERS;
        inputs.velocityMetersPerSecond = leftVelocity.getValueAsDouble() * ROTATIONS_TO_METERS;

        inputs.appliedVolts = leftAppliedVolts.getValueAsDouble();
        inputs.currentAmps = new double[] {leftCurrent.getValueAsDouble(), rightCurrent.getValueAsDouble()};
        inputs.tempCelsius = new double[] {leftTemp.getValueAsDouble(), rightTemp.getValueAsDouble()};

        // Raw rotation values for advanced logging
        inputs.leftPositionRotations = leftPosition.getValueAsDouble();
        inputs.leftVelocityRotationsPerSecond = leftVelocity.getValueAsDouble();
        inputs.rightPositionRotations = rightPosition.getValueAsDouble();
        inputs.rightVelocityRotationsPerSecond = rightVelocity.getValueAsDouble();

        // Limit switches (assuming normally open, active low)
        inputs.topLimitSwitch = !topLimit.get();
        inputs.bottomLimitSwitch = !bottomLimit.get();
        inputs.carriageDetected = !carriageSensor.get();

        // Motor connection status
        inputs.leftMotorConnected = leftPosition.getStatus().isOK();
        inputs.rightMotorConnected = rightPosition.getStatus().isOK();
    }

    @Override
    public void setVoltage(double volts) {
        leftMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setPosition(double positionMeters) {
        double positionRotations = positionMeters * METERS_TO_ROTATIONS;
        leftMotor.setControl(motionMagicRequest.withPosition(positionRotations));
    }

    @Override
    public void setBrakeMode(boolean enable) {
        var neutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        var leftConfig = new TalonFXConfiguration();
        var rightConfig = new TalonFXConfiguration();

        leftMotor.getConfigurator().refresh(leftConfig);
        rightMotor.getConfigurator().refresh(rightConfig);

        leftConfig.MotorOutput.NeutralMode = neutralMode;
        rightConfig.MotorOutput.NeutralMode = neutralMode;

        leftMotor.getConfigurator().apply(leftConfig.MotorOutput);
        rightMotor.getConfigurator().apply(rightConfig.MotorOutput);
    }

    @Override
    public void resetEncoder() {
        leftMotor.setPosition(0.0);
        rightMotor.setPosition(0.0);
    }

    @Override
    public void configureMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
        var motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = cruiseVelocity * METERS_TO_ROTATIONS;
        motionMagicConfig.MotionMagicAcceleration = acceleration * METERS_TO_ROTATIONS;
        motionMagicConfig.MotionMagicJerk = jerk * METERS_TO_ROTATIONS;

        leftMotor.getConfigurator().apply(motionMagicConfig);
    }
}
