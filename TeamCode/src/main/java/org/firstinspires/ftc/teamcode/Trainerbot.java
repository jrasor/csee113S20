/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single
 * robot, a Trainerbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot"
 * for usage examples easily converted to run on a Trainerbot.
 * <p>
 * This robot class assumes the following device names have been configured on
 * the robot, meaning they are present in the robot configuration file:
 * <p>
 * Motor channel:  Left  drive motor:       "motor0"
 * Motor channel:  Right drive motor:       "motor1"
 * Servo channel:  Servo to move paddle:    "paddle"
 * Sensor channel:	colorS sensor:						"colorSensor"
 */

// Version history
// v 0.1	JMR initial class for CSEE331, Fall 2018.
//
// v 0.2	JMR 7/7/19 extended with members from 2017-2018 Runnerbot.
// v 0.3  JMR 7/9/19 common movement and paddle operations moved to Trainerbot
//				as class members. Unused opmodes dumped.
// v 0.31	JMR 7/13/19	added color sensor support.

public class Trainerbot extends GenericFTCRobot {
	// Trainerbot specific drive train members.
	static final double COUNTS_PER_MOTOR_REV = 1120; // REV NeveRest 40 motor.
	static final double DRIVE_GEAR_REDUCTION = 1.0;
	public DcMotor leftDrive = null;   // Motor Port 0 on REV motor hub
	public DcMotor rightDrive = null;   // Motor Port 1 on REV motor hub

	// Trainerbot specific measurements in inches.
	static final double WHEEL_DIAMETER_INCHES = 2.9; // was 3.0;
	static final double DRIVE_WHEEL_SEPARATION = 15.25; // was 14.875;
	static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
			(WHEEL_DIAMETER_INCHES * Math.PI);
	// ** To do: adjust for a Trainerbot. Start with zero, camera facing center of Blue rover image
	final int CAMERA_FORWARD_DISPLACEMENT = 0;   // eg: Camera 110 mm in front of robot center
	final int CAMERA_VERTICAL_DISPLACEMENT = 0;   // eg: Camera 200 mm above Field
	final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera right on the robot's center line

	// Trainerbot specific actuator members.
	public Servo paddle = null;
	public static final double STOWED = 1.0;
	public static final double DEPLOYED = 0.0;

	// Trainerbot specific sensor members.
	public ColorSensor colorSensor;

	/* local OpMode members. */
	HardwareMap hwMap = null;
	//private ElapsedTime period = new ElapsedTime();

	/* Constructors */
	public Trainerbot() {
		super ();
	};
	public Trainerbot(LinearOpMode linearOpMode) {
		//super (linearOpMode);
		currentOpMode = linearOpMode;
	}

	/*
	*										Drive Train methods
	*/

	/* Full motor initialization. */
	// Do this by initializing various aspects using methods below.
	public void initMotors(HardwareMap ahwMap) {
		// Save reference to Hardware map
		hwMap = ahwMap;

		// Define Motors
		leftDrive= hwMap.get(DcMotor.class, "motor0");
		rightDrive = hwMap.get(DcMotor.class, "motor1");
		setDriveDirections();
		setDriveRunMode (DcMotor.RunMode.RUN_USING_ENCODER);
		setDrivetrainPower (0.0);
		setDriveStopBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	/*  Initialization on various motor properties. */
	//  Set directions on both drive motors, enabling tank drive.
	public void setDriveDirections() {
		leftDrive.setDirection(DcMotor.Direction.REVERSE); // Opposite on Runnerbot
		rightDrive.setDirection(DcMotor.Direction.FORWARD);
	}

	// Initialize both drive motors to some RunMode.
	public void setDriveRunMode(DcMotor.RunMode someRunMode) {
		leftDrive.setMode(someRunMode);
		rightDrive.setMode(someRunMode);
	}
	// Set both drive motors to some behavior when they're told to stop.
	public void setDriveStopBehavior(DcMotor.ZeroPowerBehavior someBehavior) {
		leftDrive.setZeroPowerBehavior(someBehavior);
		rightDrive.setZeroPowerBehavior(someBehavior);
	}

	public void initializePaddle (double somePosition) {
		paddle = hwMap.get(Servo.class, "paddle");
		paddle.setPosition(somePosition);
	}

	public void initializeColorSensor (boolean stateOfLED){
		// ColorSensor colorSensor;
		colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
		// Turn the LED on.
		colorSensor.enableLed(stateOfLED);
	}
	/*  Initialize hardware by groups. */
	public void initHardware(HardwareMap ahwMap) {
		// Save reference to Hardware map
		hwMap = ahwMap;


		// Define and Initialize Motors
		//rightDrive = hwMap.get(DcMotor.class, "motor0");
		//leftDrive = hwMap.get(DcMotor.class, "motor1");
		//setDriveDirections();
		initMotors(hwMap);
		// Set all motors to zero power
		//leftDrive.setPower(0);
		//rightDrive.setPower(0);

		// Set both motors to run with encoders.
		//setDriveRunMode (DcMotor.RunMode.RUN_USING_ENCODER);
		//leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		//rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		// Define and initialize installed servo. If you add others,
		// initialize them here.
		initializePaddle (STOWED);
		///paddle = hwMap.get(Servo.class, "paddle");
		//paddle.setPosition(STOWED);

		initializeColorSensor (true);
	}

	/* Initialize standard drive train equipment. */
	public void initUnencodedDrive() {
		// Define and Initialize Motors
		initMotors (hwMap);
		//setDriveDirections();

		// Call initEncodedDrive if encoders are installed.
		// Generally, Autonomous modes will run encoded, and Driver (TeleOp) modes
		//   will run unencoded.
		setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	/*      Initialize drive equipment to use encoders.         */
	public void initEncodedDrive() {
		// Define and Initialize Motors
		initMotors (hwMap);
		//setDriveDirections();
		setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}


	/*              Grand Autonomous Encoded Initializer        */
	public void initAutoHardware (HardwareMap someMap) {
		hwMap = someMap;
		initMotors(hwMap);
		// Stop all motors
		setDriveStopBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftDrive.setPower(0);
		rightDrive.setPower(0);
		leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

	public void resetEncoderDrive() {
		// Zero the encoder counts targets. Side effect is to remove
		//   power, but we will do that explicitly.
		setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

	/**************************************************************************
	 * Robot movement members.
	 **************************************************************************/

	/*
	 *  Hardware primitives layer, the most primitive movement members.
	 *  These send direct commands to the hardware.
	 */

	/*  General movement. Most other movement methods will be wrappers for this.
	 *    Specify speed and end condition for both motor pairs. Move will
	 *    stop if either of two conditions occur:
	 *  1) One of the two drive motor pairs gets to the desired position.
	 *  2) Driver quits the opmode.
	 */

	public void encoderDrive(double leftSpeed, double rightSpeed,
													 double leftInches, double rightInches) {
		int newLeftTarget;
		int newRightTarget;

		//  Discard current encoder positions.
		setDriveStopBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		// Determine new target positions, and pass to motor controller.
		newLeftTarget =  (int) (leftInches * COUNTS_PER_INCH);
		newRightTarget = (int) (rightInches * COUNTS_PER_INCH);
		leftDrive.setTargetPosition(newLeftTarget);
		rightDrive.setTargetPosition(newRightTarget);

		// Turn On RUN_TO_POSITION
		setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

		// Go!
		leftDrive.setPower(Math.abs(leftSpeed));
		rightDrive.setPower(Math.abs(rightSpeed));

		// keep looping while we are still active, and both motors are running.
		while (leftDrive.isBusy() && rightDrive.isBusy()) {
			// Wait until motors done before doing anything else.
		}
		// Clean up, prepare for next segment.
		leftDrive.setPower(0);
		rightDrive.setPower(0);

		// Turn off RUN_TO_POSITION
		leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void stopDriveMotors(){
		leftDrive.setPower(0);
		rightDrive.setPower(0);
	}

	public void fullPowerDrive () {
		leftDrive.setPower(1.0);
		rightDrive.setPower(1.0);
	}

	public void setDrivetrainPower(double power){
		leftDrive.setPower(power);
		rightDrive.setPower(power);
	}
	/*                  Robot vision: OpenCV                        */
	private LinearOpMode currentOpMode;
	public OpenGLMatrix lastLocation = null;

	//  Command layer. Human driver issues commands with gamepad.
	public void justDrive (){
		//  Tank drive with the two sticks.
		double leftSpeed = -currentOpMode.gamepad1.left_stick_y;
		double rightSpeed = -currentOpMode.gamepad1.right_stick_y;
		//  Spin on axis with right sick, x motion.
		//double rightX = -currentOpMode.gamepad1.right_stick_x;
		//  Not tempered.
		leftDrive.setPower(leftSpeed);
		rightDrive.setPower(rightSpeed);
	}

	public void operatePaddle () {
		if (currentOpMode.gamepad1.y) {
			paddle.setPosition(DEPLOYED);
		};
		if (currentOpMode.gamepad1.a) {
			paddle.setPosition(STOWED);
		};
	}
	/*
	 *  All other movement members are at the command layer.
	 */

	//  This one requires no command layer to hardware layer translation.
	//  Just continue going straight.
	public void continueStraight(double speed) {
		leftDrive.setPower(speed);
		rightDrive.setPower(speed);
	}

	//   Simple wrapper for encoderDrive. Just go straight a number of inches.
	public void driveStraight(double speed, double inches){
		encoderDrive(speed, speed, inches, inches);
	}

	//   Turn on axis, as though with left and right tank drive joysticks in equal but
	// opposite deflection.
	public void turnAngle (double speed, double angle) { // angle in radians
		double inches = angle * DRIVE_WHEEL_SEPARATION/2;
		encoderDrive (speed, speed, -inches, inches);
	}

	/*  Turning movements. All angles are in radians. */
	//  Turn at speed through an angle, with a given radius.
	public void turnAngleRadiusDrive(double speed, double angle, double radius) {

		// One or both turning arcs could be negative.
		// Degenerate cases: angle = 0, R = 0, R = d/2, R = +infinity
		// (straight drive).
		// Calculate 2 target distances, 2 speeds. Then feed 'em to encoderDrive.
		// Arc lengths are angle * radius adjusted for the drive wheels: one shorter,
		// the other longer. Speeds need to be adjusted as well.
		// TODO: handle 4 quadrant cases: forward CW, forward CCW, back CW, back CCW
		// ** Note negative angle enforces backward movement. What would negative
		// angle and negative radius do?
		double leftRadius = radius - DRIVE_WHEEL_SEPARATION / 2.0;
		double rightRadius = radius + DRIVE_WHEEL_SEPARATION / 2.0;
		double cfLeft = leftRadius / radius;
		double cfRight = rightRadius / radius;
		double leftArc = leftRadius * angle;
		double rightArc = rightRadius * angle;
		double leftSpeed = speed * cfLeft;
		double rightSpeed = speed * cfRight;

		encoderDrive(leftSpeed, rightSpeed, leftArc, rightArc);
		// clean up
	}

	//    Wrapper for turnAngleRadius
	// ** make this a wrapper for encoderDrive instead.
	public void turnArcRadiusDrive(double speed, double arc, double radius) {
		double targetAngle = arc / radius;
		turnAngleRadiusDrive(speed, targetAngle, radius);
	}

	//  Begin a left turn at speed, sharpness of turn decided by ratio.
	// ** Test on Meet 3 build.
	//    1:  go straight.
	//    0:  turn axis is left wheel.
	//    -1: turn axis is between drive wheels. Robot turns on own axis.
	public void steerLeft (double speed, double ratio) {
		Range.clip(ratio, -1.0, 1.0);
		leftDrive.setPower(speed * ratio);
		rightDrive.setPower(speed);
	}

	//  Right analog of steerLeft.
	public void steerRight(double speed, double ratio) {
		Range.clip(ratio, -1.0, 1.0);
		leftDrive.setPower(speed);
		rightDrive.setPower(speed * ratio);
	}

	//  Drive a curved path by making left wheels turn slower and go
	//    shorter path by a factor of ratio. The right wheels will spin
	//    at parameter speed, and travel the full arc.
	public void turnLeft (double speed, double ratio, double arcInches) {
		Range.clip(ratio, -1.0, 1.0);
		encoderDrive(speed*ratio, speed,
				arcInches*ratio, arcInches);
	}

	//  Right analog of turnLeft.
	public void turnRight (double speed, double ratio, double arcInches) {
		Range.clip(ratio, -1.0, 1.0);
		encoderDrive(speed, speed*ratio,
				arcInches, arcInches*ratio);
	}
}

