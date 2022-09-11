package com.team5940.frc2022.statemachines;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team5940.lib.math.BreadUtil;
import com.team5940.lib.statemachine.Request;
import com.team5940.lib.statemachine.StateMachine;

import edu.wpi.first.math.MathUtil;

public class Shooter extends StateMachine {
    
    // Hardware
    private final TalonFX kLeftFlywheelMotor = new TalonFX(13);
    private final TalonFX kRightFlywheelMotor = new TalonFX(14);
    private final CANSparkMax kHoodMotor = new CANSparkMax(15, MotorType.kBrushless);
    private final RelativeEncoder kHoodEncoder = kHoodMotor.getEncoder();
    private final SparkMaxPIDController kHoodPID = kHoodMotor.getPIDController();
    private final SparkMaxAnalogSensor kHoodLimit = kHoodMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);

    // Constants
    private final double kFlywheelGearing = 19.0/36.0;
    private final double kHoodGearing = (12/60.0)*(20.0/460.0);
    private final TalonFXInvertType kLeftMotorDir = TalonFXInvertType.CounterClockwise;
    private final TalonFXInvertType kRightMotorDir = TalonFXInvertType.OpposeMaster;
    private final double kMinHoodTravel = 0.0;
    private final double kMaxHoodTravel = 37.928627014160156;


    // State Logic Variables
    private SystemState mSystemState = SystemState.HOMING;
    private double mStateStartTime = 0;
    private boolean mWantsHome = false;
    private boolean mWantsShoot = false;
    private double mHoodSetpoint = 0.0;
    private double mFlywheelSetpoint = 0.0; 
    private final double kStabalizationDelta = 0.25;

    enum SystemState {
        HOMING, IDLE, APPROACHING_SETPOINT, STABALIZING, AT_SETPOINT
    }

    public Shooter() {
        // Configure left flywheel motor
        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.slot0.kP = integratedSensorUnitsToFlywheelRPM(0.0015) * 1023.0;
        leftMotorConfig.slot0.kI = integratedSensorUnitsToFlywheelRPM(0.0) * 1023.0;
        leftMotorConfig.slot0.kD = integratedSensorUnitsToFlywheelRPM(0.0) * 1023.0;
        leftMotorConfig.slot0.kF = 0.0;
        leftMotorConfig.slot0.closedLoopPeakOutput = 1.0;
        leftMotorConfig.peakOutputForward = 1.0;
        leftMotorConfig.peakOutputReverse = 0.0;
        leftMotorConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_10Ms;
        leftMotorConfig.velocityMeasurementWindow = 8;
        leftMotorConfig.voltageCompSaturation = 9.5;
        leftMotorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 80, 80, 1.5);
        kLeftFlywheelMotor.selectProfileSlot(0, 0);
        kLeftFlywheelMotor.setInverted(kLeftMotorDir);
        kLeftFlywheelMotor.enableVoltageCompensation(true);
        kLeftFlywheelMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        kLeftFlywheelMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);

        // Configure right flywheel motor
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        rightMotorConfig.peakOutputForward = 1.0;
        rightMotorConfig.peakOutputReverse = -0.0;
        rightMotorConfig.voltageCompSaturation = 9.5;
        rightMotorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 80, 80, 1.5);
        kRightFlywheelMotor.setInverted(kRightMotorDir);
        kRightFlywheelMotor.follow(kLeftFlywheelMotor);
        kRightFlywheelMotor.enableVoltageCompensation(true);
        kRightFlywheelMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 239);
        kRightFlywheelMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 233);

        // Configure hood motor 
        kHoodEncoder.setPositionConversionFactor(kHoodGearing * 360.0);
        kHoodPID.setFeedbackDevice(kHoodEncoder);
        kHoodPID.setP(0.3);
        kHoodPID.setI(0.0);
        kHoodPID.setD(0.05);
        kHoodPID.setOutputRange(-0.4, 0.4);
    }

    @Override
    protected void init() {
        mStateStartTime = BreadUtil.getFPGATimeSeconds();
    }

    @Override
    protected void handle(Request r) throws RuntimeException {
        if (r instanceof RequestHome) {
            mWantsHome = true;
        } else if (r instanceof RequestShoot) {
            RequestShoot rs = (RequestShoot)r;
            if (rs.kFlywheelVel == 0.0) {
                mWantsShoot = false;
            } else {
                mWantsShoot = true;
                mFlywheelSetpoint = rs.kFlywheelVel;
                mHoodSetpoint = rs.kHoodPos;
            }
        } else if (r instanceof RequestIdle) {
            mWantsShoot = false;
        } else {
            throw new RuntimeException("Must input a valid shooter request!");
        }
    }

    @Override
    protected void onLoop() {
        // Outputs
        if (mSystemState == SystemState.HOMING) {
            commandHoodVoltage(-1.5);
            commandFlywheelVelocity(0.0);
        } else if (mSystemState == SystemState.IDLE) {
            commandHoodPosition(10.0);
            commandFlywheelVelocity(1000.0);
        } else if (mSystemState == SystemState.APPROACHING_SETPOINT) {
            commandHoodPosition(mHoodSetpoint);
            commandFlywheelVelocity(mFlywheelSetpoint);
        } else if (mSystemState == SystemState.STABALIZING) {
            commandHoodPosition(mHoodSetpoint);
            commandFlywheelVelocity(mFlywheelSetpoint);
        } else if (mSystemState == SystemState.AT_SETPOINT) {
            commandHoodPosition(mHoodSetpoint);
            commandFlywheelVelocity(mFlywheelSetpoint);
        }

        // Transitions
        SystemState nextState = mSystemState;
        if (mSystemState == SystemState.HOMING) {
            if (getHoodLimitSwitchTriggered()) {
                resetHood(0.0);
                nextState = SystemState.IDLE;
            }
        } else if (mSystemState == SystemState.IDLE) {
            if (mWantsHome) {
                nextState = SystemState.HOMING;
            } else if (mWantsShoot) {
                nextState = SystemState.APPROACHING_SETPOINT;
            }
        } else if (mSystemState == SystemState.APPROACHING_SETPOINT) {
            if (mWantsHome) {
                nextState = SystemState.HOMING;
            } else if (!mWantsShoot) {
                nextState = SystemState.IDLE;
            } else if (flywheelAtSetpoint()&&hoodAtSetpoint()) {
                nextState = SystemState.STABALIZING;
            }
        } else if (mSystemState == SystemState.STABALIZING) {
            if (mWantsHome) {
                nextState = SystemState.HOMING;
            } else if (!mWantsShoot) {
                nextState = SystemState.IDLE;
            } else if (BreadUtil.getFPGATimeSeconds() - mStateStartTime >= kStabalizationDelta) {
                nextState = SystemState.AT_SETPOINT;
            }
        } else if (mSystemState == SystemState.AT_SETPOINT) {
            if (mWantsHome) {
                nextState = SystemState.HOMING;
            } else if (!mWantsShoot) {
                nextState = SystemState.IDLE;
            } else if (!flywheelAtSetpoint()||!hoodAtSetpoint()) {
                nextState = SystemState.STABALIZING;
            }
        }

        if (nextState != mSystemState) {
            mSystemState = nextState;
            mStateStartTime = BreadUtil.getFPGATimeSeconds();
        }
    }

    @Override
    protected void log() {
        
    }

    /* HELPER METHODS */

    // Converts integrated sensor units to flyweel rpm
    private double integratedSensorUnitsToFlywheelRPM(double integratedSensorUnits) {
        return integratedSensorUnits * ((600/2048.0) * kFlywheelGearing);
    }

    // Converts flywheel rpm to integrated sensor units
    private double flywheelRPMToIntegratedSensorUnits(double flywheelRPM) {
        return flywheelRPM / ((600/2048.0) * kFlywheelGearing);
    }

    // Private method to reset the hood encoder
    private void resetHood(double newPosition) {
        kHoodEncoder.setPosition(newPosition);
    }

    // Returns whether the hood is at its setpoint
    public boolean hoodAtSetpoint() {
        return BreadUtil.atReference(getHoodPosition(), getHoodSetpoint(), 0.5, true);
    }

    // Returns whether the flywheel is at its setpoint
    public boolean flywheelAtSetpoint() {
        return BreadUtil.atReference(getFlywheelVelocity(), getFlywheelSetpoint(), 50.0, true);
    }

    /* SETTERS (commands) */

    // Private method to set the hood setpoint
    private void commandHoodPosition(double degrees) {
        double adjustedSetpoint = MathUtil.clamp(degrees, kMinHoodTravel + 1, kMaxHoodTravel - 1);
        if ((getHoodLimitSwitchTriggered() && degrees < getHoodPosition())) {
            kHoodMotor.set(0.0);
        } else {
            kHoodPID.setReference(adjustedSetpoint, CANSparkMax.ControlType.kPosition);
        }
    }

    // Private method to set the hood percent
    private void commandHoodVoltage(double volts) {
        kHoodMotor.setVoltage(volts);
    }

    // Private method to set the flywheel setpoint
    private void commandFlywheelVelocity(double rpm) {
        if (rpm==0.0) {
            kLeftFlywheelMotor.set(ControlMode.PercentOutput, 0.0);
        } else {
            kLeftFlywheelMotor.set(ControlMode.Velocity, flywheelRPMToIntegratedSensorUnits(rpm), DemandType.ArbitraryFeedForward, FeedForwardInterpolatingTable.get((rpm-50.0)/kFlywheelGearing));
        }
    }

    /* GETTERS */

    // Returns the current state of the shooter
    public SystemState getSystemState() {
        return mSystemState;
    }

    // Returns the hood position
    public double getHoodPosition() {
        return kHoodEncoder.getPosition();
    }

    // Returns the hood velocity
    public double getHoodVelocity() {
        return kHoodEncoder.getVelocity();
    }

    // Returns the voltage of the hood limit switch
    public boolean getHoodLimitSwitchTriggered() {
        return kHoodLimit.getVoltage() < 1.5;
    }

    // Returns the flywheel velocity
    public double getFlywheelVelocity() {
        return integratedSensorUnitsToFlywheelRPM(kLeftFlywheelMotor.getSelectedSensorVelocity());
    }

    // Returns the hood setpoint
    public double getHoodSetpoint() {
        return mHoodSetpoint;
    }

    // Returns the flywheel setpoint
    public double getFlywheelSetpoint() {
        return mFlywheelSetpoint;
    }

    /* REQUESTS */
    class RequestHome extends Request {

        public RequestHome() {
            super(Shooter.this);
        }

    }

    class RequestShoot extends Request {

        public final double kFlywheelVel;
        public final double kHoodPos;

        public RequestShoot(double flywheelVel, double hoodPos) {
            super(Shooter.this);
            kFlywheelVel = flywheelVel;
            kHoodPos = hoodPos;        
        }
    }

    class RequestIdle extends Request {

        public RequestIdle() {
            super(Shooter.this);
        }
    }
    
}
