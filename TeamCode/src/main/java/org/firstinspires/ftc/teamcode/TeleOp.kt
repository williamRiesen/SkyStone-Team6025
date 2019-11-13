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


@TeleOp(name = "TeleOp", group = "TurtleDozer")
class TeleOp : OpMode() {

    val TWO_PI = PI * 2.0
    val ROTATION_SPEED_ADJUST = 0.25

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
        if (gamepad2.dpad_down) {
            robot.tailhook.position = 0.0
        }

        if (gamepad2.dpad_up) {
            robot.tailhook.position = 0.5
        }

        val rotation = gamepad1.left_stick_x.toDouble()
        val xScooch = gamepad1.right_stick_x.toDouble()
        val yScooch = -gamepad1.right_stick_y.toDouble()
        val heading = robot.motionSensor.getHeading()

        telemetry.addData("Heading", heading)
        telemetry.addData("Bearing", bearing)
        telemetry.addData("Relative Bearing", relativeBearing)
        telemetry.addData("Rotation", rotation)
        telemetry.update()

        val driveCommand = DriveCommand(xScooch, yScooch, rotation * ROTATION_SPEED_ADJUST)
        driveCommand.rotate(heading)
        robot.setDriveMotion(driveCommand)
    }

    override fun stop() {
        with(robot) {
            stopAllMotors()
//            visualLocalizer.close()
            motionSensor.imu.stopAccelerationIntegration()
        }
    }
}