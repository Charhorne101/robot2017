package org.usfirst.frc.team5679.robot;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

/**
 * @author Robotics
 *
 */
public class Robot extends IterativeRobot {
	private static final int RIGHT_AXIS = 5;
	private static final int LEFT_AXIS = 1;
	private static final int B_BUTTON_ID = 2;
	private static final int A_BUTTON_ID = 1;
	private static final int LEFT_BUMPER_ID = 5;
	private static final int RIGHT_BUMPER_ID = 6;
	private static final int LEFT_TRIGGER_ID = 2;
	private static final int RIGHT_TRIGGER_ID = 3;
	private static final int X_BUTTON_ID = 3;
	private static final int Y_BUTTON_ID = 4;
	private static final double CLAW_OPEN_CLOSE_SPEED = 0.25;
	private static final double SCISSOR_LIFT_SPEED = .7;
	private static final double SCISSOR_LIFT_MAX = 1000;
	private static final double SCISSOR_LIFT_OFFSET = 10;
	private static final double CLAW_RAISE_LOWER_SPEED = 0.25;
	
	Talon leftMotor0 = new Talon(0);
	Talon leftMotor1 = new Talon(1);
	Talon rightMotor0 = new Talon(2);
	Talon rightMotor1 = new Talon(3);
	Talon leftScissorLiftActuator = new Talon(4);
	Talon rightScissorLiftActuator = new Talon(5);
	Talon tiltClawActuator = new Talon(6);
	
	DigitalInput limitSwitchLiftTop = new DigitalInput(6);
	DigitalInput limitSwitchLiftBottom = new DigitalInput(7);
	
	Joystick driveJoystick = new Joystick(0);
	SpeedControllerGroup m_left = new SpeedControllerGroup(leftMotor0, leftMotor1);
	SpeedControllerGroup m_right = new SpeedControllerGroup(rightMotor0, rightMotor1);
	DifferentialDrive drive = new DifferentialDrive(m_left, m_right);
	
	SpeedControllerGroup scissorLift = new SpeedControllerGroup(leftScissorLiftActuator, rightScissorLiftActuator);

	Encoder rightEncoder = new Encoder(2, 3, false, EncodingType.k4X);
	Encoder leftEncoder = new Encoder(0, 1, false, EncodingType.k4X);
	
	CameraServer camera;
	int session;	

	static final double wheelCircumference = 1.43;
	static final double encoderPulses = 250;
	static final double distancePerPulse = wheelCircumference / encoderPulses;
	static final double speedFactor = -1;
	static final double driveOffset = .98;

	static final double halfSpeed = .5;
	static final double minJoystickValue = 0.2;
	static final double minimumSpeed = 0.1;
	static final int fullSpeed = 1;
	static final double motorExpiration = .2;
	static final double autonomousDistance = 11;
	static final double autonomousSpeed = .4;
	static final double retrogradeSpeed = -.2;
	double speedAdjust = .8;

	// Defining which panels are exactly where on the field.
	PanelDirection homeSwitchDirection;
	PanelDirection opponentSwitchDirection;
	PanelDirection middleScaleDirection;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		SmartDashboard.putString("Autonomous", "Robot Init");
		leftMotor0.setExpiration(motorExpiration);
		leftMotor1.setExpiration(motorExpiration);
		rightMotor0.setExpiration(motorExpiration);
		rightMotor1.setExpiration(motorExpiration);
		leftScissorLiftActuator.setExpiration(motorExpiration);
		tiltClawActuator.setExpiration(motorExpiration);
		clawActuator.setExpiration(motorExpiration);

		rightEncoder.setDistancePerPulse(distancePerPulse);
		leftEncoder.setDistancePerPulse(distancePerPulse);

		SmartDashboard.putString("robot init", "robot init");

		rightEncoder.reset();
		leftEncoder.reset();				
	}	

	/**
	 * This function sets up any necessary data before the autonomous control loop.
	 */
	public void autonomousinit() {
		SmartDashboard.putString("Autonomous", "Init");
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.charAt(0) == 'L') {
			// TODO: Define buttons to control scissor lift actuator and finish autonomous
			// code
			// ((Object) leftScissorliftActuator).whenPressed (new Lift());
			// rightScissorliftActuator.whenPressed(new Lift());

			homeSwitchDirection = PanelDirection.Left;
		} else {
			homeSwitchDirection = PanelDirection.Right;
		}
		if (gameData.charAt(1) == 'L') {
			middleScaleDirection = PanelDirection.Left;
		} else {
			middleScaleDirection = PanelDirection.Right;
		}
		if (gameData.charAt(2) == 'L') {
			opponentSwitchDirection = PanelDirection.Left;
		} else {
			opponentSwitchDirection = PanelDirection.Right;
		}
		rightEncoder.reset();
		leftEncoder.reset();
	}

	public void debug() {
		SmartDashboard.putNumber("Joystick x", driveJoystick.getX());
		SmartDashboard.putNumber("Joystick y", driveJoystick.getY());
		SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
		SmartDashboard.putNumber("Left Encoder", leftEncoder.getDistance());
		SmartDashboard.putNumber("Automous Distance Limit", autonomousDistance);
		SmartDashboard.putBoolean("Autonomous Right Distance Triggered", rightEncoder.getDistance() >= autonomousDistance);
		SmartDashboard.putBoolean("Autonomous Left Distance Triggered", leftEncoder.getDistance() >= autonomousDistance);
	}

	/**
	 * This function is called periodically during autonomous control
	 */
	@Override
	public void autonomousPeriodic() {
		debug();
				
		if (Math.abs(rightEncoder.getDistance()) >= autonomousDistance || Math.abs(leftEncoder.getDistance()) >= autonomousDistance) {
			SmartDashboard.putString("Autonomous", "Stop");
            drive.tankDrive(retrogradeSpeed, retrogradeSpeed);
            drive.tankDrive(0, 0);
			//drive.stopMotor();
		}
		else {
			setRobotDriveSpeed(autonomousSpeed, autonomousSpeed);
			SmartDashboard.putString("Autonomous", "Go");
		}
	}
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putString("Autonomous", "Teleop");
		SmartDashboard.putBoolean("limit lift top",limitSwitchLiftTop.get());
		SmartDashboard.putBoolean("limit lift bottom", limitSwitchLiftBottom.get());		
		rightEncoder.reset();
		leftEncoder.reset();
		double LP = driveJoystick.getRawAxis(LEFT_AXIS);
		double RP = driveJoystick.getRawAxis(RIGHT_AXIS);

		if (driveJoystick.getRawAxis(RIGHT_TRIGGER_ID) > minJoystickValue) {
			//if (!limitSwitchBottom.get()) {
				lowerClaw(CLAW_RAISE_LOWER_SPEED);
			//}
			SmartDashboard.putString("Right Trigger", "Pressed");
		} else {

			SmartDashboard.putString("Right Trigger", "Not Pressed");
		}
		if (driveJoystick.getRawAxis(LEFT_TRIGGER_ID) > minJoystickValue) {

			SmartDashboard.putString("Left Trigger", "Pressed");
			// Send negative scissor lift speed to lower scissor lift

			if (limitSwitchLiftBottom.get()) {
				moveScissorLift(SCISSOR_LIFT_SPEED * -1);
			}
		} else {

			SmartDashboard.putString("Left Trigger", "Not Pressed");
		}
		if (driveJoystick.getRawButton(RIGHT_BUMPER_ID)) {
			//if (!limitSwitchTop.get()) {
				raiseClaw(CLAW_RAISE_LOWER_SPEED);
			//}

			SmartDashboard.putString("Right Bumper", "Pressed");
		} else {

			SmartDashboard.putString("Right Bumper", "Not Pressed");
		}
		if (driveJoystick.getRawButton(LEFT_BUMPER_ID)) {

			SmartDashboard.putString("Left Bumper", "Pressed");
			if (limitSwitchLiftTop.get()) {
				moveScissorLift(SCISSOR_LIFT_SPEED * 1);
			}

		} else {

			SmartDashboard.putString("Left Bumper", "Not Pressed");
		}

		if (driveJoystick.getRawButton(A_BUTTON_ID)) {
			openClaw(CLAW_OPEN_CLOSE_SPEED);
			SmartDashboard.putString("A BUTTON ", "Pressed");
		} else {

			SmartDashboard.putString("A BUTTON", "Not Pressed");
		}
		if (driveJoystick.getRawButton(B_BUTTON_ID)) {
			closeClaw(CLAW_OPEN_CLOSE_SPEED);
			SmartDashboard.putString("B BUTTON", "Pressed");
		} else {

			SmartDashboard.putString("B BUTTON", "Not Pressed");
		}
		if (Math.abs(RP) < minimumSpeed) {
			RP = 0;

			if (Math.abs(LP) < minimumSpeed) {
				LP = 0;
			}
		}

		if (driveJoystick.getRawButton(Y_BUTTON_ID)) {
			speedAdjust = halfSpeed;
		}

		setRobotDriveSpeed(-RP * speedAdjust, -LP * speedAdjust);

	}

	/**
	 * This method sets the speed and applies the limiting speed factor for
	 * SpeedControllers (motor)
	 * 
	 * @param motor
	 *            the motor for which we are setting the speed.
	 * @param speed
	 *            to which we are setting the motor (base speed)
	 */
	public void setMotorSpeed(SpeedController motor, double speed) {
		motor.set(speed * speedFactor);
	}

	/**
	 * This method sets the speed and applies the limiting speed factor for robot
	 * Tank Drive
	 * 
	 * @param leftSpeed
	 * @param rightSpeed
	 */
	public void setRobotDriveSpeed(double leftSpeed, double rightSpeed) {
		SmartDashboard.putNumber("leftspeed", leftSpeed);
		SmartDashboard.putNumber("rightspeed", rightSpeed);

		drive.tankDrive(leftSpeed * speedFactor, rightSpeed * speedFactor);
	}

	public void moveScissorLift(double speed) {
		scissorLift.set(speed);
	}

	public void openClaw(double speed) {
		setMotorSpeed(clawActuator, speed);
	}

	public void closeClaw(double speed) {
		setMotorSpeed(clawActuator, -1 * speed);
	}

	public void raiseClaw(double speed) {
		setMotorSpeed(tiltClawActuator, speed);
	}

	public void lowerClaw(double speed) {
		setMotorSpeed(tiltClawActuator, -1 * speed);
	}
}

enum PanelDirection {
	Right, Left
}