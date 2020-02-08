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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single
 * robot, a Runnerbot. An ancestor of thsi robot was the competition robot
 * for FTC 5197 in "Relic Recovery", and will be the competition robot for
 * FTC 16546 for "SkyStone".
 *
 * The hardware is a heavily modified AndyMark TileRunner.
 *
 * This robot class has no servos or sensors except for the Robot Controller
 * phone. It can carry a phone around and look at stuff. Actuators and methods
 * for them will be added later.
 *
 * version history:
 * v 0.1 jmr modified from csee331 SkyStone code.
 */

public class Runnerbot extends GenericFTCRobot
{
    // Runnerbot specific measurements
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ; // AndyMark NeveRest40
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;

    // Runnerbot specific drive train members.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     DRIVE_WHEEL_SEPARATION  = 17.0 ;
    static final double     COUNTS_PER_INCH         =
        (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    // Runnerbot specific motor and actuator members.
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;

    /* local OpMode members. */
    HardwareMap hwMap           =   null;
    private ElapsedTime period  =   new ElapsedTime();

    /* Constructor */
    public Runnerbot(){
        super();
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Tetrix motors
        rightDrive  = hwMap.get(DcMotor.class, "motor0");
        leftDrive = hwMap.get(DcMotor.class, "motor1");
        // Set to REVERSE if using AndyMark motors
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        // Set to FORWARD if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        setDrivetrainPower (0.0);
        setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDrivetrainPower (double somePower) {
        leftDrive.setPower(somePower);
        rightDrive.setPower(somePower);
    }

    // Initialize both drive motors to some RunMode.
    public void setDriveRunMode(DcMotor.RunMode someRunMode) {
        leftDrive.setMode(someRunMode);
        rightDrive.setMode(someRunMode);
    }
}
