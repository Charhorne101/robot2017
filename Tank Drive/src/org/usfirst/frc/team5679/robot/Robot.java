package org.usfirst.frc.team5679.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	Talon leftMotor0 = new Talon(0);
	Talon leftMotor1 = new Talon(1);
	Talon rightMotor0 = new Talon(2);
	Talon rightMotor1 = new Talon(3);
	// @TODO check firing controller port
	Talon fuelFiringController = new Talon(4);
	// @TODO check spark controller port number
	Spark sparkFuelCollectorController = new Spark(6);
	Joystick driveJoystick = new Joystick(0);
	Joystick firingJoystick = new Joystick(1);
	RobotDrive drive = new RobotDrive(leftMotor0, leftMotor1, rightMotor0, rightMotor1);
	DigitalInput releaseFuelLimitSwitch = new DigitalInput(0);
	DigitalInput closeFuelLimitSwitch = new DigitalInput(5);
	//AnalogGyro gyro = new AnalogGyro(0);
	BuiltInAccelerometer accel = new BuiltInAccelerometer();
	Encoder rightEncoder = new Encoder(3, 4, false, EncodingType.k4X);
	Encoder leftEncoder = new Encoder(1, 2, false, EncodingType.k4X);
	CameraServer camera;

	// Adjust this value down for more distance in autonomous, up for less distance
	static final double wheelCircumference = 1.43;
	static final double startingAngle = 0;
	static final double Kp = .02;
	static final double speedFactor = .75;
	static final double firingSpeedFactor = 1;
	static final double driveOffset = .98;
	static final double encoderPulses = 250;
	static final double distancePerPulse = wheelCircumference / encoderPulses;
	static final double halfSpeed = .5;
	static final double minJoystickValue = 0.2;
	static final double minimumSpeed = 0.1;
	static final double waterWheelSpeed = 1;
	static final double autonomousSpeed = .7;
	static final double releaseFuelSpeed = 1;
	static final double closeFuelSpeed = 1;
	static final double slowerSpeed = .85;
	// @TODO: Set firing arm button
	static final int holdFiringArmButton = 2;
	static final int fullSpeed = 1;
	static final int firingJoystickAxis = 0;
	static final int driveJoystickLeftAxis = 1;
	static final int driveJoystickRightAxis = 5;
	static final int releaseFuelButton = 1;
	static final int closeFuelButton = 3;
	static final int joystickCameraButton = 5;
	static final int autonomousDistance = 16;
	static final String cameraImageFileName = "/camera/image.jpg";
	static final int frontCameraName = 0;
	static final int rearCameraName = 1;
	static final String frontCameraDescription = "Front";
	static final String rearCameraDescription = "Rear";
	static final GenericHID.Hand waterWheelButton = GenericHID.Hand.kRight;
	static double speedAdjust = 1;
	int activeCameraName = frontCameraName;
	String activeCameraDescription = frontCameraDescription;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture(frontCameraName);

		rightEncoder.setDistancePerPulse(distancePerPulse);
		leftEncoder.setDistancePerPulse(distancePerPulse);

		SmartDashboard.putString("robot init", "robot init");

		rightEncoder.reset();
		leftEncoder.reset();

		SmartDashboard.putString("Camera", activeCameraDescription);
	}

	/**
	 * This function switches between two cameras.
	 */
	public void changeCamera() {
		if (activeCameraName == frontCameraName) {
			activeCameraName = rearCameraName;
			activeCameraDescription = rearCameraDescription;
		} else {
			activeCameraName = frontCameraName;
			activeCameraDescription = frontCameraDescription;
		}

		SmartDashboard.putString("Camera", activeCameraDescription);

		CameraServer.getInstance().startAutomaticCapture(activeCameraName);
	}

	/**
	 * This function sets up any necessary data before the autonomous control
	 * loop.
	 */
	public void autonomousinit() {
		rightEncoder.reset();
		leftEncoder.reset();
		SmartDashboard.putString("autonomous init", "autonomous init");
	}

	/**
	 * This function is called periodically during autonomous control
	 */
	@Override
	public void autonomousPeriodic() {
		moveBase(autonomousDistance, autonomousSpeed, 0);

		debug();
	}

	public void debug() {
		// SmartDashboard.putNumber("AccelX", accel.getX());
		// SmartDashboard.putNumber("AccelY", accel.getY());
		// SmartDashboard.putNumber("AccelZ", accel.getZ());
		// SmartDashboard.putNumber("Joystick x", driveJoystick.getX());
		// SmartDashboard.putNumber("Joystick y", driveJoystick.getY());
		// SmartDashboard.putBoolean("Limit", firingLimitSwitch.get());
		// SmartDashboard.putNumber("Right Encoder",
		// rightEncoder.getDistance());
		// SmartDashboard.putNumber("Left Encoder", -1 *
		// leftEncoder.getDistance());
	}

	/**
	 * This function is for moving forward a set number of feet. Returns a
	 * boolean indicating whether the movement is complete.
	 */
	public boolean moveBase(double feet, double speed, double angle) {
		double tankDriveMin = -.2;
		if (rightEncoder.getDistance() >= feet || leftEncoder.getDistance() >= feet) {
			drive.tankDrive(tankDriveMin, tankDriveMin);
			drive.tankDrive(0, 0);
			return true;
		} else {
			drive.tankDrive(speed, speed * driveOffset);
			return false;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		rightEncoder.reset();
		leftEncoder.reset();
		double LP = -driveJoystick.getRawAxis(driveJoystickLeftAxis);
		double RP = -driveJoystick.getRawAxis(driveJoystickRightAxis);

		if (driveJoystick.getRawAxis(3) > minJoystickValue) {
			speedAdjust = fullSpeed;
		} else if (driveJoystick.getRawAxis(2) > minJoystickValue) {
			speedAdjust = halfSpeed;
		} else {
			speedAdjust = slowerSpeed;
		}

		if (Math.abs(LP) < minimumSpeed) {
			LP = 0;

			if (Math.abs(RP) < minimumSpeed) {
				RP = 0;
			}
		}

		setRobotDriveSpeed(drive, LP * speedAdjust, RP * speedAdjust);

		if (driveJoystick.getTrigger(waterWheelButton)) {
			rotateWaterWheel(waterWheelSpeed);
		}

		if (firingJoystick.getRawButton(releaseFuelButton)) {
			releaseFuel(releaseFuelSpeed);
		}
		if (firingJoystick.getRawButton(closeFuelButton)) {
			closeFuel(closeFuelSpeed);
		}

		boolean cameraButton = driveJoystick.getRawButton(joystickCameraButton);

		if (cameraButton) {
			changeCamera();
		}
	}

	/**
	 * This method sets the speed and applies the limiting speed factor for
	 * Talons
	 * 
	 * @param motor
	 * @param speed
	 */
	public void setTalonSpeed(Talon motor, double speed) {
		motor.set(speed * speedFactor);
	}

	/**
	 * Sets the speed on a spark motor
	 * @param motor
	 * @param speed
	 */
	public void setSparkSpeed(Spark motor, double speed) {
		motor.set(speed * speedFactor);
	}

	/**
	 * This method sets the speed and applies the limiting speed factor for
	 * robot Tank Drive
	 * 
	 * @param driveTrain
	 * @param leftSpeed
	 * @param rightSpeed
	 */
	public void setRobotDriveSpeed(RobotDrive driveTrain, double leftSpeed, double rightSpeed) {
		driveTrain.tankDrive(leftSpeed * speedFactor, rightSpeed * speedFactor);
	}

	/**
	 * Sets motor controller speed to specified value
	 * 
	 * @param speed must be between 1 and -1 (backwards)
	 */
	public void rotateWaterWheel(double speed) {
		setSparkSpeed(sparkFuelCollectorController, speed);
	}

	/**
	 * Opens the fuel releaser.
	 * 
	 * @param speed must be between 1 and -1 (backwards)
	 */
	public void releaseFuel(double speed) {
		rotateFuelRelease(speed);
	}

	/**
	 * Sets fuel release motor controller speed to specified value until limit
	 * switch is hit
	 * 
	 * @param speed must be between 1 and -1 (backwards)
	 */
	public void rotateFuelRelease(double speed) {
		if (speed > 0) {
			if (releaseFuelLimitSwitch.get()) {
				setTalonSpeed(fuelFiringController, speed);
			} else {
				setTalonSpeed(fuelFiringController, 0);
			}
		} else if (speed < 0) {
			if (closeFuelLimitSwitch.get()) {
				setTalonSpeed(fuelFiringController, speed);
			} else {
				setTalonSpeed(fuelFiringController, 0);
			}
		} else {
			setTalonSpeed(fuelFiringController, speed);
		}
	}

	/**
	 * Closes the fuel releaser.
	 * 
	 * @param speed must be between 1 and -1 (backwards)
	 */
	public void closeFuel(double speed) {
		rotateFuelRelease(-speed);
	}
}
