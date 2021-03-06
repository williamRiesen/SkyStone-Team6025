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
import kotlin.math.atan2

@TeleOp(name = "HolobotTeleOp", group = "Holobot")
class HoloBotTeleOp : OpMode() {

    lateinit var robot: HoloBot

    override fun init() {
        robot = HoloBot(hardwareMap)
        robot.setLights(GREEN)
    }

    override fun loop() {
        val xInput = gamepad1.right_stick_x
        val yInput = gamepad1.right_stick_y
        val rotationInput = gamepad1.left_stick_x
        val driverCommand = DriveCommand(xInput, yInput, rotationInput)

        val locationMatrix = robot.visualLocalizer.getLocation()
        if (locationMatrix != null) {
            val x = locationMatrix[0, 0]
            val y = locationMatrix[0, 1]
            val heading = atan2(y, x)
            driverCommand.rotate(heading)
        } else {
            val heading = robot.motionSensor.getHeading()
            driverCommand.rotate(heading)
        }
        robot.setDriveMotion(driverCommand)
    }

    override fun stop() {
        with(robot) {
            stopAllMotors()
            visualLocalizer.close()
            motionSensor.imu.stopAccelerationIntegration()
        }
    }
}