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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single
 * robot, a Tablebot.
 *
 * A Tablebot is just a bunch of parts scattered on a table, and trainees assemble
 * them according to the REV Robotics Wiring Reference Sheet.
 * <p>
 * A Tablebot assumes the following device names have been configured on
 * it, meaning they are present in the robot configuration file:

 * <p>
 * Motor channel:	"motor0"
 * Servo channel:	"servo0"
 * DigitalDevice: 	"touchsensor0"
 * I2C device:      "colorSensor0"
 */

// Version history
// v 0.1	JMR initial class for CSEE331, Fall 2019.

public class Tablebot extends GenericFTCRobot {
	// Trainerbot specific drive train members.
	LinearOpMode currentOpMode;
	static final double COUNTS_PER_MOTOR_REV = 1680; // REV NeveRest 60 motor.
	static final double DRIVE_GEAR_REDUCTION = 1.0;
	public DcMotor motor = null;   // Motor Port 0 on REV motor hub

	// Tablebot specific actuator members.
	public Servo servo = null;
	public static final double CENTERED = 0.5;

	// Tablebot specific sensor members.
	public TouchSensor touchSensor;

	/* local OpMode members. */
	HardwareMap hwMap = null;
	//private ElapsedTime period = new ElapsedTime();

	/* Constructors */
	public Tablebot() {
		super ();
	};
	public Tablebot(LinearOpMode linearOpMode) {
		//super (linearOpMode);
		currentOpMode = linearOpMode;
	}

	/*

	*										Drive Train methods
	*
	* A Tablebot has only one motor.
	*/

	/* Full motor initialization. */
	// Do this by initializing various aspects using methods below.
	public void initMotor(HardwareMap ahwMap) {
		// Save reference to Hardware map
		hwMap = ahwMap;

		// Define Motor
		motor = hwMap.get(DcMotor.class, "motor0");
		setDriveDirection();
		setMotorRunMode (DcMotor.RunMode.RUN_USING_ENCODER);
		setMotorPower (0.0);
		setMotorStopBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	/*  Initialization on various motor properties. */
	//  Set directions on both drive motors, enabling tank drive.
	public void setDriveDirection() {
		motor.setDirection(DcMotor.Direction.REVERSE);
	}

	// Initialize motor to some RunMode.
	public void setMotorRunMode(DcMotor.RunMode someRunMode) {
		motor.setMode(someRunMode);
	}

	// Set motor to some behavior when they're told to stop.
	public void setMotorStopBehavior(DcMotor.ZeroPowerBehavior someBehavior) {
		motor.setZeroPowerBehavior(someBehavior);
	}


	public void initializeTouchSensor (){
		// ColorSensor colorSensor;
		touchSensor = hwMap.get(TouchSensor.class, "touchSensor0");
	}

	/*  Initialize hardware by groups. */
	public void initHardware(HardwareMap ahwMap) {
		// Save reference to Hardware map
		hwMap = ahwMap;

		initMotor(hwMap);

		// Define and initialize installed servo. If you add others,
		// initialize them here.
		servo.setPosition(CENTERED);
		//initializeServo (CENTERED);
		///paddle = hwMap.get(Servo.class, "paddle");
		//paddle.setPosition(STOWED);

		initializeTouchSensor();
	}

	/* Initialize standard drive train equipment. */
	public void initUnencodedDrive() {
		// Define and Initialize Motors
		initMotor (hwMap);
		setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	/*      Initialize motor to use encoders.         */
	public void initEncodedDrive() {
		// Define and Initialize Motors
		initMotor (hwMap);
		setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}


	/*              Grand Autonomous Encoded Initializer        */
	public void initAutoHardware (HardwareMap someMap) {
		hwMap = someMap;
		initMotor(hwMap);
		// Stop all motors
		setMotorStopBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motor.setPower(0);
		motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

	public void resetEncoderDrive() {
		// Zero the encoder counts targets. Side effect is to remove
		//   power, but we will do that explicitly.
		setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

	/**************************************************************************
	 * Motor movement members.
	 **************************************************************************/

	/*
	 *  Hardware primitives layer, the most primitive movement members.
	 *  These send direct commands to the hardware.
	 */

	/*  General movement. Most other movement methods will be wrappers for this.
	 *    Specify speed and end condition for motor. Move will stop if either of
	 *   two conditions occurs:
	 *  1) Motor gets to the desired position.
	 *  2) Driver quits the opmode.
	 */

	public void encoderDrive(double speed, double turns) {
		int newTarget;

		//  Discard current encoder positions.
		setMotorStopBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		// Determine new target positions, and pass to motor controller.
		newTarget =  (int) (turns * COUNTS_PER_MOTOR_REV);
		motor.setTargetPosition(newTarget);

		// Turn On RUN_TO_POSITION
		setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

		// Go!
		motor.setPower(Math.abs(speed));

		// keep looping while we are still active, and both motors are running.
		while (motor.isBusy() ) {
			// Wait until motor is done before doing anything else.
		}
		// Clean up, prepare for next segment.
		motor.setPower(0);

		// Turn off RUN_TO_POSITION
		motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void stopMotor(){
		motor.setPower(0);
	}

	public void fullPowerDrive () {
		motor.setPower(1.0);
	}

	public void setMotorPower(double power){
		motor.setPower(power);
	}

	//  Command layer. Human driver issues commands with gamepad.
	public void justDrive (){
		//  Tank drive with the two sticks.
		double speed = -currentOpMode.gamepad1.right_stick_y;
		//  Spin on axis with right sick, x motion.
		//double rightX = -currentOpMode.gamepad1.right_stick_x;
		//  Not tempered.
		motor.setPower(speed);
	}

}

