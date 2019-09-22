/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters
import java.util.*

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.
 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 *
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


class VisualLocalizer(hwMap: HardwareMap) {
    // Class Members
    private var lastLocation: OpenGLMatrix? = null
    //    private var vuforia: VuforiaLocalizer? = null
    private var targetVisible = false
    private var phoneXRotate = 0f
    private var phoneYRotate = 0f
    private val phoneZRotate = 0f
    private lateinit var targetsSkyStone: VuforiaTrackables
    val allTrackables: MutableList<VuforiaTrackable> = ArrayList()

    init {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */

        val cameraMonitorViewId = hwMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.packageName)
        val parameters = Parameters(cameraMonitorViewId)

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY
        parameters.cameraDirection = CAMERA_CHOICE
        val vuforia = ClassFactory.getInstance().createVuforia(parameters)

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone")
        val stoneTarget: VuforiaTrackable = targetsSkyStone[0]
        stoneTarget.name = "Stone Target"
        val blueRearBridge: VuforiaTrackable = targetsSkyStone[1]
        blueRearBridge.name = "Blue Rear Bridge"
        val redRearBridge: VuforiaTrackable = targetsSkyStone[2]
        redRearBridge.name = "Red Rear Bridge"
        val redFrontBridge: VuforiaTrackable = targetsSkyStone[3]
        redFrontBridge.name = "Red Front Bridge"
        val blueFrontBridge: VuforiaTrackable = targetsSkyStone[4]
        blueFrontBridge.name = "Blue Front Bridge"
        val red1: VuforiaTrackable = targetsSkyStone[5]
        red1.name = "Red Perimeter 1"
        val red2: VuforiaTrackable = targetsSkyStone[6]
        red2.name = "Red Perimeter 2"
        val front1: VuforiaTrackable = targetsSkyStone[7]
        front1.name = "Front Perimeter 1"
        val front2: VuforiaTrackable = targetsSkyStone[8]
        front2.name = "Front Perimeter 2"
        val blue1: VuforiaTrackable = targetsSkyStone[9]
        blue1.name = "Blue Perimeter 1"
        val blue2: VuforiaTrackable = targetsSkyStone[10]
        blue2.name = "Blue Perimeter 2"
        val rear1: VuforiaTrackable = targetsSkyStone[11]
        rear1.name = "Rear Perimeter 1"
        val rear2: VuforiaTrackable = targetsSkyStone[12]
        rear2.name = "Rear Perimeter 2"

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsSkyStone)

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of *transformation matrices.*
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See [Transformation Matrix](https://en.wikipedia.org/wiki/Transformation_matrix)
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the [OpenGLMatrix] class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         * - The X axis runs from your left to the right. (positive from the center to the right)
         * - The Y axis runs from the Red Alliance Station towards the other side of the field
         * where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         * - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         * coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms


        stoneTarget.location = OpenGLMatrix
                .translation(0f, 0f, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90f, 0f, -90f))

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.location = OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0f, bridgeRotY, bridgeRotZ))
        blueRearBridge.location = OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0f, -bridgeRotY, bridgeRotZ))
        redFrontBridge.location = OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0f, -bridgeRotY, 0f))
        redRearBridge.location = OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0f, bridgeRotY, 0f))

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.location = OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90f, 0f, 180f))
        red2.location = OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90f, 0f, 180f))
        front1.location = OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90f, 0f, 90f))
        front2.location = OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90f, 0f, 90f))
        blue1.location = OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90f, 0f, 0f))
        blue2.location = OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90f, 0f, 0f))
        rear1.location = OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90f, 0f, -90f))
        rear2.location = OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90f, 0f, -90f))

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.


        phoneYRotate = if (CAMERA_CHOICE == BACK) {
            -90f
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        else {
            90f
        }

        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90f
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.


        val CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch   // eg: Camera is 4 Inches in front of robot center

        val CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch   // eg: Camera is 8 Inches above ground

        val CAMERA_LEFT_DISPLACEMENT = 0f     // eg: Camera is ON the robot's center line

        val robotFromCamera: OpenGLMatrix? = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate))

        /**  Let all the trackable listeners know where the phone is.   */

        for (trackable in allTrackables) {
            (trackable.listener as VuforiaTrackableDefaultListener).setPhoneInformation(robotFromCamera!!, parameters.cameraDirection)
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.


        targetsSkyStone.activate()
    }

    public fun getLocation(): OpenGLMatrix? {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false
        for (trackable in allTrackables) {
            if ((trackable.listener as VuforiaTrackableDefaultListener).isVisible) {
//                telemetry.addData("Visible Target", trackable.name)
                targetVisible = true
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                val robotLocationTransform: OpenGLMatrix? = (trackable.listener as VuforiaTrackableDefaultListener).updatedRobotLocation
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform
                }
                break
            }
        }

        // Provide feedback as to where the robot is located (if we know).

        return lastLocation


//        if (targetVisible) {
//            // express position (translation) of robot in inches.
//
//            val translation: VectorF = lastLocation!!.translation
//            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch)
//
//            // express the rotation of the robot in degrees.
//
//
//            val rotation: Orientation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES)
//            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle)
//        } else {
//            telemetry.addData("Visible Target", "none")
//        }
//        telemetry.update()
    }

    public fun close() {
        // Disable Tracking when we are done;
        targetsSkyStone.deactivate()
    }

    companion object {
        // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
        // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
        // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
        //
        // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
        //
        private val CAMERA_CHOICE = BACK
        private const val PHONE_IS_PORTRAIT = false
        /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
        private const val VUFORIA_KEY = " -- YOUR NEW VUFORIA KEY GOES HERE  --- "
        // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
        // We will define some constants and conversions here
        private const val mmPerInch = 25.4f
        private const val mmTargetHeight = 6 * mmPerInch          // the height of the center of the target image above the floor

        // Constant for Stone Target
        private const val stoneZ = 2.00f * mmPerInch
        // Constants for the center support targets
        private const val bridgeZ = 6.42f * mmPerInch
        private const val bridgeY = 23 * mmPerInch
        private const val bridgeX = 5.18f * mmPerInch
        private const val bridgeRotY = 59f                                 // Units are degrees

        private const val bridgeRotZ = 180f
        // Constants for perimeter targets
        private const val halfField = 72 * mmPerInch
        private const val quadField = 36 * mmPerInch
    }
}