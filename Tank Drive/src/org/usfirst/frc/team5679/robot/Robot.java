package org.usfirst.frc.team5679.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
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

/**
 * @author Robotics
 *
 */
public class Robot extends IterativeRobot {
	private static final int REVERSE_WATERWHEEL_AXIS = 2;
	private static final int WATERWHEEL_AXIS = 3;
	private static final int RIGHT_AXIS = 5;
	private static final int LEFT_AXIS = 1;
	private static final int B_BUTTON_ID = 2;
	private static final int A_BUTTON_ID = 1;
	private static final int LEFT_BUMPER_ID = 5;
	private static final int RIGHT_BUMPER_ID = 6;
	private static final int X_BUTTON_ID = 3;
	Talon leftMotor0 = new Talon(0);
	Talon leftMotor1 = new Talon(1);
	Talon rightMotor0 = new Talon(2);
	Talon rightMotor1 = new Talon(3);
	Servo fuelDumpServo = new Servo(5);
	
	Spark fuelCollectorController = new Spark(4);
	Joystick driveJoystick = new Joystick(2);
	RobotDrive drive = new RobotDrive(leftMotor0, leftMotor1, rightMotor0,
			rightMotor1);
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
	static final double speedFactor = 1;
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
	static final double waterWheelSpeed = -1;
	static final double waterWheelStop = 0;
	static final double firingMaxDistance = 1;
	static final String imageFileName = "/camera/image.jpg";
	static final double fuelDumpAngle = 90;
	static final double closeFuelHatchAngle = 0;
	static final double motorExpiration=.2;
	static final double autonomousDistance = 4;
	static final double autonomousSpeed = .5;
	static final double retrogradeSpeed = -.2;
		
	double speedAdjust = 1;
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
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		leftMotor0.setExpiration(motorExpiration);
		leftMotor1.setExpiration(motorExpiration);
		rightMotor0.setExpiration(motorExpiration);
		rightMotor1.setExpiration(motorExpiration);
		fuelCollectorController.setExpiration(motorExpiration);
		CameraServer.getInstance().startAutomaticCapture(activeCameraName, activeCameraNumber);
		
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
		
		switch (autonomousMode) {
			// Mode 0, drive
			case 0:
				nextStep = moveBase(autonomousDistance, -autonomousSpeed);
				
				break;
			// Mode 1, Drive and fire
			case 1: 
				switch (stepToPerform) {
					case 0:
						// adjust the first number in the movebase call for number of feet to move in autonomous
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
	 * This function is for moving forward a set number of feet. Returns a
	 * boolean indicating whether the movement is complete.
	 */
	public boolean moveBase(double feet, double speed) {
		if (rightEncoder.getDistance() >= feet
				|| leftEncoder.getDistance() >= feet) {
			drive.tankDrive(retrogradeSpeed, retrogradeSpeed);
			drive.tankDrive(0, 0);
			return true;
		}
		
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
		double LP = -driveJoystick.getRawAxis(LEFT_AXIS);
		double RP = -driveJoystick.getRawAxis(RIGHT_AXIS);
			
		if (driveJoystick.getRawAxis(WATERWHEEL_AXIS) > minJoystickValue){
			
			rotateWaterWheel(waterWheelSpeed);

			SmartDashboard.putString("Left Bumper", "Pressed");
		}
		else if (driveJoystick.getRawAxis(REVERSE_WATERWHEEL_AXIS) > minJoystickValue) {
			rotateWaterWheel(-waterWheelSpeed);
		}
		else {
		
			SmartDashboard.putString("Left Bumper", "Not Pressed");
			rotateWaterWheel(waterWheelStop);
		}
		
		if (Math.abs(LP) < minimumSpeed) {
			LP = 0;

			if (Math.abs(RP) < minimumSpeed) {
				RP = 0;
			}
		}
		
		
		//@TODO set fuel collector to joystick button
		if (driveJoystick.getRawButton(LEFT_BUMPER_ID))
		{
			speedAdjust = fullSpeed;
		}
		else if (driveJoystick.getRawButton(RIGHT_BUMPER_ID)){
			speedAdjust = halfSpeed;
		}
		
		setRobotDriveSpeed(drive, LP * speedAdjust, RP * speedAdjust);
		
		if (driveJoystick.getRawButton(A_BUTTON_ID)){
			dumpFuel();
		}
		else if (driveJoystick.getRawButton(B_BUTTON_ID)){
			closeFuelHatch();
		}
	}

	/**
	 * This method sets the speed and applies the limiting speed factor for
	 * SpeedControllers (motor)
	 * 
	 * @param motor the motor for which we are setting the speed.
	 * @param speed to which we are setting the motor (base speed)
	 */
	public void setMotorSpeed(SpeedController motor, double speed) {
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

		SmartDashboard.putNumber("leftspeed", leftSpeed);
		SmartDashboard.putNumber("rightspeed", rightSpeed);
		SmartDashboard.putNumber("leftencoder", leftEncoder.getDistance());
		SmartDashboard.putNumber("rightencoder", rightEncoder.getDistance());
		
		
		driveTrain.tankDrive(leftSpeed * speedFactor, rightSpeed * speedFactor);
	}

	/**
	 * This method dumps fuel.
	 * @return when the servo is turned to a certain degree
	 */
	public boolean dumpFuel(){
		turnServo(fuelDumpServo, fuelDumpAngle);
		return true;
	}
	
	/**
	 * this method closes the fuel hatch.
	 * @return when the hatch is closed 
	 */
	public boolean closeFuelHatch(){
		turnServo(fuelDumpServo, closeFuelHatchAngle);
		return true; 
	}
	
	/**
	 * Sets motor controller speed to specified value
	 * @param speed must be between 1 and -1 (backwards)
	 * @return true when the waterwheel is successfully turning
	 */
	public boolean rotateWaterWheel (double speed){
		setMotorSpeed(fuelCollectorController, speed);
		return true;
	}
	
	/**
	 * This method turns the servo to a certain angle.
	 * @param servo the specific servo that we are going to turn.
	 * @param angle how far we are going to turn the servo. 
	 */
	public void turnServo(Servo servo, double angle){
		servo.setAngle(angle);
	}
	
	/**
	 * This function switches the camera from front to rear.
	 * This function might cause an error.
	 */
	public void bamboozleCamera(){
		if (activeCameraName == frontCameraName){
			activeCameraName = rearCameraName;
			activeCameraNumber = rearCameraNumber;
		}
		else {
			activeCameraName = frontCameraName;
			activeCameraNumber = frontCameraNumber; 
		}
		CameraServer.getInstance().startAutomaticCapture(activeCameraName, activeCameraNumber);
	}
}
