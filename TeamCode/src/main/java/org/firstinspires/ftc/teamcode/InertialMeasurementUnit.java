/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification,
 * are permitted (subject to the limitations in the disclaimer below)
 * provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or
 * promote products derived from this software without specific prior written
 * permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Developed by https://stemrobotics.cs.pdx.edu/node/7266, adapted by Team
 * 5197, and
 * distributed under the above license.
 */

package org.firstinspires.ftc.teamcode;

// Simple autonomous program that drives bot forward until end of 30s period
// or gamepad button is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates use of the REV Hub's built in IMU in
// place of a gyro. Also uses gamepad1 buttons to command left as well as
// right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@Autonomous(name = "Interruptible Straight with Imu", group = "Exercises")
//@Disabled
// TODO: convert to TeleOp.
public class InertialMeasurementUnit extends LinearOpMode {
  Trainerbot              robot   = new Trainerbot(this);
  private BNO055IMU imu;
  private Orientation lastAngles = new Orientation();
  private double globalAngle = 0.0;

  // called when init button is  pressed.
  @Override
  public void runOpMode() throws InterruptedException {
    robot.initHardware(hardwareMap);
    robot.setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    double power = .30;
    double correction = 0.0;
    boolean xButton, bButton;

    robot.setDriveDirections();
    robot.setDriveStopBehavior(BRAKE);

    // TODO: make BNO055IMU a generic robot property. Any conceivable robot
    // will use a REV hub with one of those things.
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    parameters.mode = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled = false;

    // Retrieve and initialize the IMU. We expect the IMU to be attached to
    // an I2C port on a Core Device Interface Module, configured to be a
    // sensor of type "AdaFruit IMU" and named "imu".
    imu = hardwareMap.get(BNO055IMU.class, "imu");

    imu.initialize(parameters);

    telemetry.addData("Mode", "calibrating...");
    telemetry.update();

    // make sure the imu gyro is calibrated before continuing.
    while (!isStopRequested() && !imu.isGyroCalibrated()) {
      sleep(50);
      idle();
    }

    telemetry.addData("Mode", "waiting for start");
    telemetry.addData("imu calib status",
        imu.getCalibrationStatus().toString());
    telemetry.update();

    waitForStart();

    telemetry.addData("Mode", "running");
    telemetry.update();
    sleep(1000);

    // drive until end of 30 second Autonomous period.
    while (opModeIsActive()) {
      // Use gyro to drive in a straight line.
      correction = checkDirection();

      telemetry.addData("1 imu heading", lastAngles.firstAngle);
      telemetry.addData("2 global heading", globalAngle);
      telemetry.addData("3 correction", correction);
      telemetry.update();

      robot.leftDrive.setPower(power - correction);
      robot.rightDrive.setPower(power + correction);

      // We record the sensor values because we will test them in more than
      // one place with time passing between those places. See the lesson on
      // Timing Considerations: https://stemrobotics.cs.pdx.edu/node/7262.
      // TODO: is the logic there implemented here?
      xButton = gamepad1.x;
      bButton = gamepad1.b;

      if (xButton || bButton) {
        // back up a little.
        robot.driveStraight(power, -4.0);

        sleep(500);

        robot.stopDriveMotors();
        // turn 90 degrees right.
        if (bButton) {
          robot.turnAngle (power, -Math.PI/2);
        }

        // turn 90 degrees left.
        if (xButton) {
          robot.turnAngle (power, Math.PI/2);
        }
        resetAngle();
        sleep (500);
      }
    }

    robot.stopDriveMotors();
  }

  /**
   * Resets the cumulative angle tracking to zero.
   */
  private void resetAngle() {
    lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,
        AxesOrder.ZYX, AngleUnit.DEGREES);
    globalAngle = 0;
  }

  /**
   * Get current cumulative angle rotation from last reset.
   *
   * @return Angle in degrees. + = left, - = right.
   */
  private double getAngle() {
    // We experimentally determined the Z axis is the axis we want to use for
    // heading angle. We have to process the angle because the imu works in
    // euler angles so the Z axis is returned as 0 to +180 or 0 to -180
    // rolling back to -179 or +179 when rotation passes 180 degrees. We
    // detect this transition and track the total cumulative angle of rotation.

    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
        AxesOrder.ZYX, AngleUnit.DEGREES);
    double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
    if (deltaAngle < -180)
      deltaAngle += 360;
    else if (deltaAngle > 180)
      deltaAngle -= 360;

    globalAngle += deltaAngle;
    lastAngles = angles;

    return globalAngle;
  }

  /**
   * See if we are moving in a straight line and if not return a power
   * correction value.
   *
   * @return Power adjustment, + is adjust left - is adjust right.
   */
  private double checkDirection() {
    // The gain value determines how sensitive the correction is to direction
    // changes. You will have to experiment with your robot to get small
    // smooth direction changes to stay on a straight line.
    // TODO: Maybe use PID control here.
    double correction, angle, gain = .05;
    angle = getAngle();
    if (angle == 0)
      correction = 0;             // no adjustment.
    else
      correction = -angle;        // reverse sign of angle for correction.
    correction = correction * gain;

    return correction;
}
  }