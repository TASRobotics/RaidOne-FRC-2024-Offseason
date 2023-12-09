package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

public class SwerveModule {
  
	private CANSparkMax m_Rotor;
	private CANSparkMax m_Throttle;

	private AbsoluteEncoder m_RotorEncoder;
	private RelativeEncoder m_ThrottleEncoder;

 	private SparkMaxPIDController m_RotorPID;
 	private SparkMaxPIDController m_ThrottlePID;

 	private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

	/**
	 * Constructs a MAXSwerveModule and configures the driving and turning motor,
	 * encoder, and PID controller. This configuration is specific to the REV
	 * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
	 * Encoder.
	 */
	public SwerveModule(int iRotorID, int iThrottleID, boolean iThrottleReversed, double iRotorOffsetAngle) {
		m_Throttle = new CANSparkMax(iThrottleID, MotorType.kBrushless);
		m_Rotor = new CANSparkMax(iRotorID, MotorType.kBrushless);

		// Factory reset, so we get the SPARKS MAX to a known state before configuring
		// them. This is useful in case a SPARK MAX is swapped out.
		m_Throttle.restoreFactoryDefaults();
		m_Rotor.restoreFactoryDefaults();

		// Setup encoders and PID controllers for the driving and turning SPARKS MAX.
		m_ThrottleEncoder = m_Throttle.getEncoder();
		m_RotorEncoder = m_Rotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
		m_ThrottlePID = m_Throttle.getPIDController();
		m_RotorPID = m_Rotor.getPIDController();
		m_ThrottlePID.setFeedbackDevice(m_ThrottleEncoder);
		m_RotorPID.setFeedbackDevice(m_RotorEncoder);

		// Apply rotor encoder offset angle
		m_RotorEncoder.setZeroOffset(iRotorOffsetAngle);

		// Apply position and velocity conversion factors for the driving encoder. The
		// native units for position and velocity are rotations and RPM, respectively,
		// but we want meters and meters per second to use with WPILib's swerve APIs.
		m_ThrottleEncoder.setPositionConversionFactor(Constants.SwerveModuleConstants.kDrivingEncoderPositionFactor);
		m_ThrottleEncoder.setVelocityConversionFactor(Constants.SwerveModuleConstants.kDrivingEncoderVelocityFactor);

		// Apply position and velocity conversion factors for the turning encoder. We
		// want these in radians and radians per second to use with WPILib's swerve
		// APIs.
		m_RotorEncoder.setPositionConversionFactor(Constants.SwerveModuleConstants.kTurningEncoderPositionFactor);
		m_RotorEncoder.setVelocityConversionFactor(Constants.SwerveModuleConstants.kTurningEncoderVelocityFactor);

		// Invert the turning encoder, since the output shaft rotates in the opposite direction of
		// the steering motor in the MAXSwerve Module.
		m_RotorEncoder.setInverted(Constants.SwerveModuleConstants.kTurningEncoderInverted);

		// Enable PID wrap around for the turning motor. This will allow the PID
		// controller to go through 0 to get to the setpoint i.e. going from 350 degrees
		// to 10 degrees will go through 0 rather than the other direction which is a
		// longer route.
		m_RotorPID.setPositionPIDWrappingEnabled(true);
		m_RotorPID.setPositionPIDWrappingMinInput(Constants.SwerveModuleConstants.kTurningEncoderPositionPIDMinInput);
		m_RotorPID.setPositionPIDWrappingMaxInput(Constants.SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput);

		// Set the PID gains for the driving motor. Note these are example gains, and you
		// may need to tune them for your own robot!
		m_ThrottlePID.setP(Constants.SwerveModuleConstants.kDrivingP);
		m_ThrottlePID.setI(Constants.SwerveModuleConstants.kDrivingI);
		m_ThrottlePID.setD(Constants.SwerveModuleConstants.kDrivingD);
		m_ThrottlePID.setFF(Constants.SwerveModuleConstants.kDrivingFF);
		m_ThrottlePID.setOutputRange(Constants.SwerveModuleConstants.kDrivingMinOutput,
		    Constants.SwerveModuleConstants.kDrivingMaxOutput);

		// Set the PID gains for the turning motor. Note these are example gains, and you
		// may need to tune them for your own robot!
		m_RotorPID.setP(Constants.SwerveModuleConstants.kTurningP);
		m_RotorPID.setI(Constants.SwerveModuleConstants.kTurningI);
		m_RotorPID.setD(Constants.SwerveModuleConstants.kTurningD);
		m_RotorPID.setFF(Constants.SwerveModuleConstants.kTurningFF);
		m_RotorPID.setOutputRange(Constants.SwerveModuleConstants.kTurningMinOutput,
		    Constants.SwerveModuleConstants.kTurningMaxOutput);	
		m_Throttle.setIdleMode(Constants.SwerveModuleConstants.kDrivingMotorIdleMode);
		m_Rotor.setIdleMode(Constants.SwerveModuleConstants.kTurningMotorIdleMode);
		m_Throttle.setSmartCurrentLimit(Constants.SwerveModuleConstants.kDrivingMotorCurrentLimit);
		m_Rotor.setSmartCurrentLimit(Constants.SwerveModuleConstants.kTurningMotorCurrentLimit);
		
		// Save the SPARK MAX configurations. If a SPARK MAX browns out during
		// operation, it will maintain the above configurations.
		m_Throttle.burnFlash();
		m_Rotor.burnFlash();
		m_desiredState.angle = new Rotation2d(m_RotorEncoder.getPosition());
		m_ThrottleEncoder.setPosition(0);

		if (RobotBase.isSimulation()) {
			REVPhysicsSim.getInstance().addSparkMax(m_Rotor, DCMotor.getNEO(1));
			REVPhysicsSim.getInstance().addSparkMax(m_Throttle, DCMotor.getNEO(1));
		}
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModuleState(m_ThrottleEncoder.getVelocity(), new Rotation2d(m_RotorEncoder.getPosition()));
	}

	/**
	 * Returns the current position of the module.
	 *
	 * @return The current position of the module.
	 */
	public SwerveModulePosition getPosition() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModulePosition(m_ThrottleEncoder.getPosition(), new Rotation2d(m_RotorEncoder.getPosition()));
	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param desiredState Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState iDesiredState) {
		// Apply chassis angular offset to the desired state.
		SwerveModuleState correctedDesiredState = new SwerveModuleState();
		correctedDesiredState.speedMetersPerSecond = iDesiredState.speedMetersPerSecond;
		
		// Optimize the reference state to avoid spinning further than 90 degrees.
		SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
		    new Rotation2d(m_RotorEncoder.getPosition()));

		// Command driving and turning SPARKS MAX towards their respective setpoints.
		m_ThrottlePID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
		m_RotorPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
		m_desiredState = iDesiredState;
	}

	/** Zeroes all the SwerveModule encoders. */
	public void resetEncoders() {
		m_ThrottleEncoder.setPosition(0);
	}

}
