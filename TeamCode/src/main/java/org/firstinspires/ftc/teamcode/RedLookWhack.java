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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Autonomous opmode for Red Alliance in the 2017-2018 FTC "Relic Recovery" game.
 * Look at right Jewel, determine its color, then make the correct robot turn to
 * knock off the Blue Jewel.
 *
 * It runs on a Trainerbot.
 */

@Autonomous(name="Red Look and Whack", group="Trainerbot")
//@Disabled
public class RedLookWhack extends LinearOpMode {

    Trainerbot              robot   = new Trainerbot(this);

    private static final double TURN_SPEED  = 0.20; // Slow: less wheel slippage
    private static final double KNOCK_ANGLE = 0.25; // radians
    private static final double DEPLOYED    = 0.04; // Extended from front
    // of robot
    private static final double STOWED      = 1.00; // Folded back toward rear

    @Override
    public void runOpMode() {
        robot.initHardware(hardwareMap);
        robot.setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean redIsRight; //  LED will help decide if this is True.

        // Wait for the game to start (driver presses PLAY).
        waitForStart();
        robot.paddle.setPosition(DEPLOYED);
        sleep(1000); // Let servo go to starting position.

        // Read the RGB data, and report colors to driver station.
        telemetry.addData("Red  ", robot.colorSensor.red());
        telemetry.addData("Green", robot.colorSensor.green());
        telemetry.addData("Blue ", robot.colorSensor.blue());
        //  TODO: handle error cases: green dominant, red == blue, no colors
        //  seen.

        //  Make and report decision.
        redIsRight = robot.colorSensor.red() > robot.colorSensor.blue();
        telemetry.addData(
            "Decision", redIsRight ? " Right is Red." : " Right is Blue.");
        telemetry.update();
        sleep (5000); // Let driver evaluate reported decision.

        if (redIsRight) { // turn CCW to knock the Blue left Jewel off
            robot.turnAngle(TURN_SPEED, KNOCK_ANGLE);
        }
        else { // turn CW to knock the Blue right Jewel off
            robot.turnAngle(TURN_SPEED, -KNOCK_ANGLE);
        }
        robot.paddle.setPosition(STOWED);
        sleep (3000); // Let driver evaluate knock action taken.
    }
}
