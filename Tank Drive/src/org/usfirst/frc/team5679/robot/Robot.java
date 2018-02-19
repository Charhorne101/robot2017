package org.usfirst.frc.team5679.robot;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
	private static final double SCISSOR_LIFT_MAX = 360;
	private static final double SCISSOR_LIFT_OFFSET = 100;
	private static final double CLAW_RAISE_LOWER_SPEED = 0.25;
	
	Talon leftMotor0 = new Talon(0);
	Talon leftMotor1 = new Talon(1);
	Talon rightMotor0 = new Talon(2);
	Talon rightMotor1 = new Talon(3);
	Talon leftScissorLiftActuator = new Talon(4);
	Talon rightScissorLiftActuator = new Talon(5);
	Talon tiltClawActuator = new Talon(6);
	Talon clawActuator = new Talon(7);

	// We aren't sure if this is the correct starting value lol (we will test it)

	Potentiometer leftScissorLiftPotentiometer = new AnalogPotentiometer(4, SCISSOR_LIFT_MAX, 0);
	Potentiometer rightScissorLiftPotentiometer = new AnalogPotentiometer(5, SCISSOR_LIFT_MAX, 0);
	DigitalInput limitSwitchTop = new DigitalInput(5);
	DigitalInput limitSwitchBottom = new DigitalInput(6);

	Joystick driveJoystick = new Joystick(0);
	SpeedControllerGroup m_left = new SpeedControllerGroup(leftMotor0, leftMotor1);
	SpeedControllerGroup m_right = new SpeedControllerGroup(rightMotor0, rightMotor1);
	DifferentialDrive drive = new DifferentialDrive(m_left, m_right);
	
	SpeedControllerGroup scissorLift = new SpeedControllerGroup(leftScissorLiftActuator, rightScissorLiftActuator);

	AnalogGyro gyro = new AnalogGyro(0);
	BuiltInAccelerometer accel = new BuiltInAccelerometer();
	Encoder rightEncoder = new Encoder(2, 3, false, EncodingType.k4X);
	int fps = 10;
	Encoder leftEncoder = new Encoder(0, 1, false, EncodingType.k4X);

	CameraServer camera;
	int session;
	// initialize default mode
	private int autonomousMode = 0;
	SendableChooser<Integer> autoChooser;
	String cameraDesc = "Front";
	

	static final double wheelCircumference = 1.43;
	static final double encoderPulses = 250;
	static final double distancePerPulse = wheelCircumference / encoderPulses;
	static final double startingAngle = 0;
	static final double Kp = .02;
	static final double speedFactor = -1;
	static final double firingSpeedFactor = 1;
	static final double driveOffset = .98;
	// Adjust this value down for more distance in autonomous, up for less distance

	static final double halfSpeed = .5;
	static final double minJoystickValue = 0.2;
	static final double minimumSpeed = 0.1;
	static final int imageQuality = 20;
	static final int fullSpeed = 1;
	static final double firingMaxDistance = 1;
	static final String imageFileName = "/camera/image.jpg";
	static final double motorExpiration = .2;
	static final double autonomousDistance = 12.834;
	/// autonomous distance is now 12.8334 feet
	static final double autonomousSpeed = .55;
	/// make autonomous speed go faster? (we will test)
	static final double retrogradeSpeed = -.2;

	double speedAdjust = .8;
	double previousFireSpeed = 0;
	boolean runOnce = true;
	boolean reverse = false;
	int stepToPerform = 0;
	long startTime;
	long fireTime = 5000;
	int cameraCount = 0;
	int cameraAttempts = 5;
	String frontCameraName = "Front Camera";
	int frontCameraNumber = 0;
	String rearCameraName = "Rear Camera";
	int rearCameraNumber = 1;
	String activeCameraName = frontCameraName;
	int activeCameraNumber = frontCameraNumber;

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
		leftMotor0.setExpiration(motorExpiration);
		leftMotor1.setExpiration(motorExpiration);
		rightMotor0.setExpiration(motorExpiration);
		rightMotor1.setExpiration(motorExpiration);
		leftScissorLiftActuator.setExpiration(motorExpiration);
		rightScissorLiftActuator.setExpiration(motorExpiration);
		tiltClawActuator.setExpiration(motorExpiration);
		clawActuator.setExpiration(motorExpiration);

		// CameraServer.getInstance().startAutomaticCapture(activeCameraName,
		// activeCameraNumber);

		rightEncoder.setDistancePerPulse(distancePerPulse);
		leftEncoder.setDistancePerPulse(distancePerPulse);

		SmartDashboard.putString("robot init", "robot init");

		rightEncoder.reset();
		leftEncoder.reset();

		autoChooser = new SendableChooser<Integer>();
		autoChooser.addDefault("Drive", 0);
		autoChooser.addObject("Drive and Fire", 1);
		SmartDashboard.putData("Autonomous mode chooser", autoChooser);
		SmartDashboard.putString("Camera", cameraDesc);
		
		
		SmartDashboard.putData("left Potent.", (Sendable) leftScissorLiftPotentiometer);
		SmartDashboard.putData("right Potent.", (Sendable) rightScissorLiftPotentiometer);
	}
	

	/**
	 * This function sets up any necessary data before the autonomous control loop.
	 */
	public void autonomousinit() {
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
		SmartDashboard.putString("autonomous init", "autonomous init");
		stepToPerform = 0;
		gyro.reset();
	}

	/**
	 * This function is called periodically during autonomous control
	 */
	@Override
	public void autonomousPeriodic() {
		autonomousMode = autoChooser.getSelected();

		boolean nextStep = false;

		switch (autonomousMode) {
		// Mode 0, drive
		case 0:
			nextStep = moveBase(autonomousDistance, autonomousSpeed);

			break;
		// Mode 1, Drive and fire
		case 1:
			switch (stepToPerform) {
			case 0:
				// adjust the first number in the movebase call for number of feet to move in
				// autonomous
				nextStep = moveBase(autonomousDistance, autonomousSpeed);
				startTime = System.currentTimeMillis();
				break;
			}

			if (nextStep) {
				stepToPerform++;
			}

			break;
		}

		debug();
	}

	public void debug() {
		SmartDashboard.putNumber("AccelX", accel.getX());
		SmartDashboard.putNumber("AccelY", accel.getY());
		SmartDashboard.putNumber("AccelZ", accel.getZ());
		SmartDashboard.putNumber("Joystick x", driveJoystick.getX());
		SmartDashboard.putNumber("Joystick y", driveJoystick.getY());
		SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
		SmartDashboard.putNumber("Left Encoder", -1 * leftEncoder.getDistance());
	}

	/**
	 * This function is for moving forward a set number of feet. Returns a boolean
	 * indicating whether the movement is complete.
	 */
	public boolean moveBase(double feet, double speed) {
		if (rightEncoder.getDistance() >= feet || leftEncoder.getDistance() >= feet) {

			drive.tankDrive(retrogradeSpeed, retrogradeSpeed);
			drive.tankDrive(0, 0);
			return true;
		}

		double angle = gyro.getAngle() * Kp;
		setRobotDriveSpeed(drive, speed, speed);
		return false;
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		rightEncoder.reset();
		leftEncoder.reset();
		double LP = driveJoystick.getRawAxis(LEFT_AXIS);
		double RP = driveJoystick.getRawAxis(RIGHT_AXIS);

		if (driveJoystick.getRawAxis(RIGHT_TRIGGER_ID) > minJoystickValue) {
			if (!limitSwitchBottom.get()) {
				lowerClaw(CLAW_RAISE_LOWER_SPEED);
			}
			SmartDashboard.putString("Right Trigger", "Pressed");
		} else {

			SmartDashboard.putString("Right Trigger", "Not Pressed");
		}
		if (driveJoystick.getRawAxis(LEFT_TRIGGER_ID) > minJoystickValue) {

			SmartDashboard.putString("Left Trigger", "Pressed");
			// Send negative scissor lift speed to lower scissor lift

			// if (scissorLiftPotentiometer.get() > SCISSOR_LIFT_OFFSET) {
			moveScissorLift(SCISSOR_LIFT_SPEED * -1);
			// }
		} else {

			SmartDashboard.putString("Left Trigger", "Not Pressed");
		}
		if (driveJoystick.getRawButton(RIGHT_BUMPER_ID)) {
			if (!limitSwitchTop.get()) {
				raiseClaw(CLAW_RAISE_LOWER_SPEED);
			}

			SmartDashboard.putString("Right Bumper", "Pressed");
		} else {

			SmartDashboard.putString("Right Bumper", "Not Pressed");
		}
		if (driveJoystick.getRawButton(LEFT_BUMPER_ID)) {

			SmartDashboard.putString("Left Bumper", "Pressed");
			// if (scissorLiftPotentiometer.get() < SCISSOR_LIFT_MAX - SCISSOR_LIFT_OFFSET)
			// {
			moveScissorLift(SCISSOR_LIFT_SPEED * 1);
			// }

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

		// @TODO set fuel collector to joystick button
//		if (driveJoystick.getRawButton(X_BUTTON_ID)) {
//			speedAdjust = fullSpeed;
//		} else 
		if (driveJoystick.getRawButton(Y_BUTTON_ID)) {
			speedAdjust = halfSpeed;
		}

		setRobotDriveSpeed(drive, -RP * speedAdjust, -LP * speedAdjust);

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
	 * @param drive2
	 * @param leftSpeed
	 * @param rightSpeed
	 */
	public void setRobotDriveSpeed(DifferentialDrive drive2, double leftSpeed, double rightSpeed) {

		SmartDashboard.putNumber("leftspeed", leftSpeed);
		SmartDashboard.putNumber("rightspeed", rightSpeed);
		SmartDashboard.putNumber("leftencoder", leftEncoder.getDistance());
		SmartDashboard.putNumber("rightencoder", rightEncoder.getDistance());

		drive2.tankDrive(leftSpeed * speedFactor, rightSpeed * speedFactor);
	}

	public void moveScissorLift(double speed) {
		scissorLift.set(speed);
	}

	/**
	 * This method turns the servo to a certain angle.
	 * 
	 * @param servo
	 *            the specific servo that we are going to turn.
	 * @param angle
	 *            how far we are going to turn the servo.
	 */
	public void turnServo(Servo servo, double angle) {
		servo.setAngle(angle);
	}

	/**
	 * This function switches the camera from front to rear. This function might
	 * cause an error.
	 */
	public void bamboozleCamera() {
		if (activeCameraName == frontCameraName) {
			activeCameraName = rearCameraName;
			activeCameraNumber = rearCameraNumber;
		} else {
			activeCameraName = frontCameraName;
			activeCameraNumber = frontCameraNumber;
		}
		CameraServer.getInstance().startAutomaticCapture(activeCameraName, activeCameraNumber);
	}

	public void openClaw(double speed) {
		setMotorSpeed(clawActuator, speed);
	}

	public void closeClaw(double speed) {
		setMotorSpeed(clawActuator, -1 * speed);
	}

	public void raiseClaw(double speed) {
		setMotorSpeed(clawActuator, speed);
	}

	public void lowerClaw(double speed) {
		setMotorSpeed(clawActuator, -1 * speed);
	}
}

enum PanelDirection {
	Right, Left
}