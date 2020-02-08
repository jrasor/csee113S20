/* Copyright (c) 2018 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static
		org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static
		org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Drive a robot around, looking at Rover Ruckus target images with the Robot Controller camera.
 * Those images allow calculation of the robot's location on the Field.
 * <p>
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This opmode combines that information with target image
 * locations are on the Field, to determine the location of the camera. <p>
 * This opmode assumes an FTC square Field configuration where the Red and Blue Alliance
 * stations are on opposite walls of each other. In future seasons, Alliance stations may be near
 * each other, with only a corner separating them.
 * <p>
 * From the Audience perspective, the Red Alliance station is on the right and the Blue Alliance
 * Station is on the left.
 * <p>
 * At kickoff, a new set of Vuforia navigation images will be released.
 * <p>
 * The four vision targets are located in the center of each of the perimeter walls with the
 * images facing inwards towards the Field center:
 * - BlueRover is the Mars Rover image target on the wall closest to the blue alliance
 * - RedFootprint is the Lunar Footprint target on the wall closest to the red alliance
 * - FrontCraters is the Lunar Craters image target on the wall closest to the audience
 * - BackSpace is the Deep Space image target on the wall farthest from the audience
 * <p>
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the Field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * explained in the Trainerbot class.
 *
 * Version history
 * 1.0		Summer 2019 JMR: developed for CSEE331 Robotics course on their Trainerbots.
 * 1.1		9/10/19 JMR Added support for Lookeebot and Tablebot. Untested.
 * 1.2		9/11/19 JMR Switched Vuforia and Tensorflow assets over to SkyStone versions.
 */

@TeleOp(name = "Trainerbot Drive Navigate Paddle", group = "Trainerbot")
//@Disabled
public class TrainerbotDriveNavPaddle extends LinearOpMode {
	Trainerbot robot = new Trainerbot(this);

	// Get Field dimensions.
	final float mmPerInch = Trainerbot.mmPerInch;
	final float mmFTCFieldWidth = Trainerbot.mmFTCFieldWidth;
	final float mmTargetHeight = Trainerbot.mmTargetHeight;

	// FRONT is the camera on the phone's screen side.
	private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

	private OpenGLMatrix lastLocation = null;
	private boolean targetVisible = false;

	@Override
	public void runOpMode() {
		robot.initHardware(hardwareMap);
		telemetry.addData("Hardware", " mapped");
		telemetry.update();
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
				"cameraMonitorViewId",
				"id", hardwareMap.appContext.getPackageName());

		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine. We
		 * can pass Vuforia the handle to a camera preview resource (on the RC phone). If no camera
		 * monitor is desired, use the parameterless constructor instead (commented out below).
		 */
		VuforiaLocalizer vuforia;
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
		// VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
		parameters.vuforiaLicenseKey = GenericFTCRobot.VUFORIA_KEY;
		parameters.cameraDirection = CAMERA_CHOICE;
		// Prevent spurious reporting on loss of tracking.
		parameters.useExtendedTracking = false;
		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		// Load the data sets that for the trackable objects. These particular data
		// sets are stored in the 'assets' part of our application.
		VuforiaTrackables targetsRoverRuckus =
				vuforia.loadTrackablesFromAsset("RoverRuckus");
		VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
		blueRover.setName("Blue-Rover");
		VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
		redFootprint.setName("Red-Footprint");
		VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
		frontCraters.setName("Front-Craters");
		VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
		backSpace.setName("Back-Space");

		// Gather all the trackable objects into a conveniently iterable collection.
		List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
		allTrackables.addAll(targetsRoverRuckus);

		/*
		 * In order for Vuforia navigation to work, we need to tell the system where each target is on
		 * the Field, and where the phone sits on the robot.  These specifications are in the form of
		 * <em>transformation matrices.</em> Transformation matrices are a central, important concept
		 * in the math here involved in localization.
		 * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
		 * for detailed information. Commonly, you'll encounter transformation matrices as instances
		 * of the {@link OpenGLMatrix} class.
		 *
		 * We also need a coordinate system.
		 * If you are standing in the Red Alliance Station looking towards the center of the Field,
		 *     - The X axis runs from your left to the right. (positive from the center to the right)
		 *     - The Y axis runs from the Red Alliance Station towards the other side of the Field
		 *       where the Blue Alliance Station is. (Positive is from the center, towards the
		 *       BlueAlliance station)
		 *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the
		 *     floor)
		 *
		 * This opmode places a Rover Ruckus specific target in the middle of each perimeter wall.
		 *
		 * Before being transformed, each target image is conceptually located at the origin of the
		 * Field's coordinate system (the center of the Field), facing up.
		 */

		/*
		 * To place the BlueRover target in the middle of the blue perimeter wall:
		 * - First we rotate it 90 around the Field's X axis to flip it upright.
		 * - Then, we translate it along the Y axis to the blue perimeter wall.
		 */
		OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
				.translation(0, mmFTCFieldWidth, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(
						EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
		blueRover.setLocation(blueRoverLocationOnField);

		/*
		 * To place the RedFootprint target in the middle of the red perimeter wall:
		 * - First we rotate it 90 around the Field's X axis to flip it upright.
		 * - Second, we rotate it 180 around the Field's Z axis so the image is flat against the red
		 * perimeter wall and facing inwards to the center of the Field.
		 * - Then, we translate it along the negative Y axis to the red perimeter wall.
		 */
		OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
				.translation(0, -mmFTCFieldWidth, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(
						EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
		redFootprint.setLocation(redFootprintLocationOnField);

		/*
		 * To place the FrontCraters target in the middle of the front perimeter wall:
		 * - First we rotate it 90 around the Field's X axis to flip it upright.
		 * - Second, we rotate it 90 around the Field's Z axis so the image is flat against the front
		 * wall and facing inwards to the center of the Field.
		 * - Then, we translate it along the negative X axis to the front perimeter wall.
		 */
		OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
				.translation(-mmFTCFieldWidth, 0, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(
						EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
		frontCraters.setLocation(frontCratersLocationOnField);

		/*
		 * To place the BackSpace target in the middle of the back perimeter wall:
		 * - First we rotate it 90 around the Field's X axis to flip it upright.
		 * - Second, we rotate it -90 around the Field's Z axis so the image is flat against the back
		 * wall and facing inwards to the center of the Field.
		 * - Then, we translate it along the X axis to the back perimeter wall.
		 */
		OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
				.translation(mmFTCFieldWidth, 0, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(
						EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
		backSpace.setLocation(backSpaceLocationOnField);

		/*
		 * Create a transformation matrix describing where the phone is on the robot.
		 *
		 * The coordinate frame for the robot looks the same as the Field.
		 * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out
		 * along the Y axis.
		 * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
		 *
		 * The phone starts out lying flat, with the screen facing Up and with the physical top of the
		 * phone pointing to the LEFT side of the Robot.  It's very important when you test this code
		 * that the top of the camera is pointing to the left side of the  robot.  The rotation angles
		 * don't work if you flip the phone.
		 *
		 * If using the rear (High Res) camera:
		 * We need to rotate the camera around it's long axis to bring the rear camera forward.
		 * This requires a negative 90 degree rotation on the Y axis
		 *
		 * If using the Front (Low Res) camera
		 * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
		 * This requires a Positive 90 degree rotation on the Y axis
		 *
		 * Next, translate the camera lens to where it is on the robot. Those dimensions are set in
		 * the Trainerbot class.
		 */

		OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
				.translation(robot.CAMERA_FORWARD_DISPLACEMENT, robot.CAMERA_LEFT_DISPLACEMENT,
						robot.CAMERA_VERTICAL_DISPLACEMENT)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
						CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

		/*  Let all the trackable listeners know where the phone is.  */
		for (VuforiaTrackable trackable : allTrackables) {
			((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(
					phoneLocationOnRobot, parameters.cameraDirection);
		}

		/* Wait for the game to begin */
		telemetry.addData(">", "Press Play to start tracking!");
		telemetry.update();
		waitForStart();

		/* Start tracking the data sets we care about. */
		targetsRoverRuckus.activate();

		while (opModeIsActive()) {
			double left;
			double right;

			robot.justDrive();
			robot.operatePaddle();

			// check all the trackable target to see which one (if any) is visible.
			targetVisible = false;
			for (VuforiaTrackable trackable : allTrackables) {
				if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
					telemetry.addData("Visible Target", trackable.getName());
					targetVisible = true;

					// getUpdatedRobotLocation() will return null if no new information is available since
					// the last time that call was made, or if the trackable is not currently visible.
					OpenGLMatrix robotLocationTransform =
							((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
					if (robotLocationTransform != null) {
						lastLocation = robotLocationTransform;
					}
					break;
				}
			}
			reportLocation();
		}
	}

	private void reportLocation () {
		// Provide feedback as to where the robot is located (if we know).
		if (targetVisible) {
			// express position (translation) of robot in inches.
			VectorF translation = lastLocation.getTranslation();
			telemetry.addData("Pos (in)",
					"      X % 6.1f           Y % 6.1f             Z % 6.1f",
					translation.get(0) / mmPerInch, translation.get(1) / mmPerInch,
					translation.get(2) / mmPerInch);

			// express the rotation of the robot in degrees.
			Orientation rotation = Orientation.getOrientation(
					lastLocation, EXTRINSIC, XYZ, DEGREES);
			telemetry.addData("Rotation",
					" Roll %6.0f°,   Pitch %6.0f°,   Heading %6.0f°",
					rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
		} else {
			telemetry.addData("I see target", "nothing.");
		}
		telemetry.update();
	}
}
