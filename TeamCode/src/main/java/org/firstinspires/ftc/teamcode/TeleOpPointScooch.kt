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

package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import kotlin.math.PI
import kotlin.math.atan2

const val THREE_PI_OVER_TWO = PI * 1.5
const val PI_OVER_TWO = 0.5 * PI
const val TWO_PI = PI * 2.0
const val FOUR_PI_OVER_SIX = 5 * PI / 6.0
const val EIGHT_PI_OVER_SIX = 8 * PI/ 6.0
const val ROTATION_SPEED_ADJUST = 0.25

@TeleOp(name = "TeleOpPointScooch", group = "TurtleDozer")
class TeleOpPointScooch : OpMode() {

    private lateinit var robot: TurtleDozer
    private var relativeBearing = 0.0
    private var bearing = 0.0
    private var rotation = 0.0

    override fun init() {
        robot = TurtleDozer(hardwareMap)
        for (motor in robot.allMotors) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
    }

    override fun loop() {
        val xStick = gamepad1.right_stick_x.toDouble()
        var yStick = -gamepad1.right_stick_y.toDouble()
        val xScooch = gamepad1.left_stick_x.toDouble()
        val yScooch = -gamepad1.left_stick_y.toDouble()
        val heading = robot.motionSensor.getHeading()


        if (xStick == 0.0 && yStick == 0.0) {
            bearing = 0.0
            rotation = 0.0

        } else {
            if (yStick == -0.0) yStick = 0.0
            bearing = atan2(xStick, yStick)

            relativeBearing = calculateRelativeBearing(bearing, heading.toDouble())

            rotation = when (relativeBearing) {
                in 0.0..PI -> relativeBearing
                else -> relativeBearing - TWO_PI
            }
        }

        telemetry.addData("Heading", heading)
        telemetry.addData("Bearing", bearing)
        telemetry.addData("Relative Bearing", relativeBearing)
        telemetry.addData("Rotation", rotation)
        telemetry.update()

//        val driveCommand = DriveCommand(xStick + xScooch, yStick + yScooch, rotation / 10.0)

        val driveCommand = DriveCommand(xStick + xScooch, yStick + yScooch, rotation * ROTATION_SPEED_ADJUST)
        driveCommand.rotate(heading)
        robot.setDriveMotion(driveCommand)

    }


    fun modulo(a: Double, b: Double) = (a % b + b) % b

    fun calculateRelativeBearing(bearing: Double, heading: Double): Double {
//        if ((bearing - heading) < -PI) {
//            return TWO_PI + bearing - heading
//        } else if ((bearing - heading) > PI) {
//            return TWO_PI - (bearing - heading)
//        } else
//            return bearing - heading
        return modulo((bearing - heading), TWO_PI)
    }


    override fun stop() {
        with(robot) {
            stopAllMotors()
//            visualLocalizer.close()
            motionSensor.imu.stopAccelerationIntegration()
        }
    }
}


//