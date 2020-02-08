package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
//import android.support.annotation.NonNull;
//import android.support.annotation.Nullable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraCalibration;
import com.vuforia.Frame;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.BlockingQueue;

/**
 * This is NOT an opmode
 *
 * This is an abstract Gerneric FTC robot class. From observation, robots do have timers to count how
 * long an opmode has been running. Not all robots use vision.
 *
 * Version history
 * ======= =======
 * v 0.1    11/02/18 @Lorenzo Pedroza. Added this javadoc
 *          7/4/19 Coach Rasor added this to CSEE 331 code. Trainerbot and Lookeebot will inherit
 *          from this.
 * v 0.11   7/6/19 Coach Rasor moved some common opmode members into here.
 */

public abstract class GenericFTCRobot {
    public ElapsedTime runTime;
    public void init(){
        runTime = new ElapsedTime();
    }

    /*                      Robot independent measurements.                   */
    // Since ImageTarget trackables use mm to specify their dimensions, we must
    // use mm for all the physical dimensions.
    // We will define some constants and conversions here
    // TODO Get these as double without other objects griping.
    public static final float mmPerInch        = 25.4f;
    // FTC Field is a square about 12' on a side.
    public static final float mmFTCFieldWidth  = (12*6) * mmPerInch;
    // Height of the center of the target images above the floor.
    public static final float mmTargetHeight   = (6) * mmPerInch;

    /*                          Vision members.                         */
    /*
     * Get a Vuforia 'Development' license key for free from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Once you have a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     *
     * You can chop it up into substrings as shown here.
     */
    public static final String VUFORIA_KEY =
            "ASkv3nr/////AAAAGYZ9CexhH0K0lDbV090F719DkwXCIXEUmExgnQNDFGjrDrk" +
            "VJnU7xNhuKHLsC32Pb1jmr+6vp6JtpVKvNmTf28ZYkUphDeajNPCLgGVxLjD6xs" +
            "fgBayqSO9bfQFeGkrdEgXlP+2oaz234afhWti9Jn8k71mzbQ4W2koX9yBMWz0YL" +
            "zUWClcasxi6Nty7SUvV+gaq3CzpKVtjKk+2EwV6ibIc0V47LAeB0lDGsGkSzuJ+" +
            "93/Ulpoj+Lwr/jbI2mu/Bs2W7U9mw73CMxvDix9o1FxyPNablla4W5C5lUDm0j2" +
            "lW5gsUNOhgvlWKQ+eCu9IBp53WbW5nfNzhXPaDDh/IlBbZuAMIJuMDEHI5PVLKT9L";

    VuforiaLocalizer vuforia;
}
