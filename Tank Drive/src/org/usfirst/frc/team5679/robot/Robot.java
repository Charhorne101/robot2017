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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;

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
	Talon talonBeltLeft = new Talon(4);
	Talon talonFiringArm = new Talon(5);
	// @TODO check spark controller port number
	Spark fuelCollectorController = new Spark(6);
	Joystick driveJoystick = new Joystick(0);
	Joystick firingJoystick = new Joystick(1);
	RobotDrive drive = new RobotDrive(leftMotor0, leftMotor1, rightMotor0,
			rightMotor1);
	DigitalInput firingLimitSwitch = new DigitalInput(0);
	AnalogGyro gyro = new AnalogGyro(0);
	BuiltInAccelerometer accel = new BuiltInAccelerometer();
	Encoder rightEncoder = new Encoder(3, 4, false, EncodingType.k4X);
	int fps = 10;
	Encoder leftEncoder = new Encoder(1, 2, false, EncodingType.k4X);

	CameraServer camera;
	int session;
	// initialize default mode
	private int autonomousMode = 0; 
	SendableChooser<Integer> autoChooser;
	String cameraDesc = "Front";
	
	static final double startingAngle = 0;
	static final double Kp = .02;
	static final double speedFactor = .75;
	static final double firingSpeedFactor = 1;
	static final double driveOffset = .98;
	// Adjust this value down for more distance in autonomous, up for less distance
	static final double wheelCircumference = 1.43;
	static final double encoderPulses = 250;
	static final double distancePerPulse = wheelCircumference / encoderPulses;
	static final double halfSpeed = .5;
	static final double minJoystickValue = 0.2;
	static final double minimumSpeed = 0.1;
	static final int imageQuality = 20;
	static final int fullSpeed = 1;
	static final double firingMaxDistance = 1;
	static final String imageFileName = "/camera/image.jpg";

	double speedAdjust = 1;
	double previousFireSpeed = 0;
	boolean runOnce = true;
	boolean reverse = false;
	int stepToPerform = 0;
	long startTime;
	long fireTime = 5000;
	int cameraCount = 0;
	int cameraAttempts = 5;

	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture();
		
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
	}

	/**
	 * This function sets up any necessary data before the autonomous control
	 * loop.
	 */
	public void autonomousinit() {
		rightEncoder.reset();
		leftEncoder.reset();
		SmartDashboard.putString("autonomous init", "autonomous init");
		stepToPerform = 0;
	}

	/**
	 * This function is called periodically during autonomous control
	 */
	@Override
	public void autonomousPeriodic() {
		autonomousMode = autoChooser.getSelected();
		
		boolean nextStep = false;
		int distance = 16;
		double speed = .7;
		switch (autonomousMode) {
			// Mode 0, drive
			case 0:
				moveBase(distance, speed, 0);
				break;
			// Mode 1, Drive and fire
			case 1: 
				switch (stepToPerform) {
					case 0:
						// adjust the first number in the movebase call for number of feet to move in autonomous
						nextStep = moveBase(distance, speed, 0);
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
//		SmartDashboard.putNumber("AccelX", accel.getX());
//		SmartDashboard.putNumber("AccelY", accel.getY());
//		SmartDashboard.putNumber("AccelZ", accel.getZ());
//		SmartDashboard.putNumber("Joystick x", driveJoystick.getX());
//		SmartDashboard.putNumber("Joystick y", driveJoystick.getY());
//		SmartDashboard.putBoolean("Limit", firingLimitSwitch.get());
//		SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
//		SmartDashboard.putNumber("Left Encoder", -1 * leftEncoder.getDistance());
	}

	/**
	 * This function is for moving forward a set number of feet. Returns a
	 * boolean indicating whether the movement is complete.
	 */
	public boolean moveBase(double feet, double speed, double angle) {
		if (rightEncoder.getDistance() >= feet
				|| leftEncoder.getDistance() >= feet) {
			drive.tankDrive(-.2, -.2);
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
		double LP = -driveJoystick.getRawAxis(1);
		double RP = -driveJoystick.getRawAxis(5);
			
		if (driveJoystick.getRawAxis(3) > minJoystickValue){
			speedAdjust = fullSpeed;
		}
		else if (driveJoystick.getRawAxis(2) > minJoystickValue) {
			speedAdjust = halfSpeed;
		}
		else {
			speedAdjust = .85;
		}
		
		if (Math.abs(LP) < minimumSpeed) {
			LP = 0;

			if (Math.abs(RP) < minimumSpeed) {
				RP = 0;
			}
		}
		
		setRobotDriveSpeed(drive, LP * speedAdjust, RP * speedAdjust);
		//@TODO set fuel collector to joystick button
		if (driveJoystick.getTrigger(GenericHID.Hand.kRight))
		{
			rotateWaterWheel(1);
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
	public void setRobotDriveSpeed(RobotDrive driveTrain, double leftSpeed,
			double rightSpeed) {
		driveTrain.tankDrive(leftSpeed * speedFactor, rightSpeed * speedFactor);
	}

	/**
	 * This method rotates the firing arm
	 * robot Tank Drive
	 * 
	 * @param speed
	 */
	public boolean rotateFiringArm(double speed){
		setTalonSpeed(talonFiringArm, speed);
		return true;
	}
	
	/**
	 * Sets motor controller speed to specified value
	 * @param speed must be between 1 and -1 (backwards)
	 * @return
	 */
	public boolean rotateWaterWheel (double speed){
		setSparkSpeed(fuelCollectorController, speed);
		return true;
	}
	
	public void zoomOut(int range) {
		throw new NotImplementedException();
	}
	
	public void zoomIn(int range){
		throw new NotImplementedException();
	}
	
	public void turnServo(int angle){
		throw new NotImplementedException();
	}
}
