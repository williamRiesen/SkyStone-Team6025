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
import kotlin.math.atan2

@TeleOp(name = "TeleOp", group = "TurtleDozer")
class TurtleDozerTeleOp : OpMode() {

    lateinit var robot: TurtleDozer

    override fun init() {
        robot = TurtleDozer(hardwareMap)
        for (motor in robot.allMotors){
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
    }


    override fun loop() {
        val xInput = gamepad1.right_stick_x.toDouble()
        val yInput = gamepad1.right_stick_y.toDouble()
        val rotationInput = gamepad1.left_stick_x.toDouble()
        val heading = robot.motionSensor.getHeading()
        telemetry.addData("Heading", heading)
        telemetry.update()
        robot.rightFrontDrive.power = (yInput + xInput) * ONE_OVER_SQRT2 + rotationInput
        robot.leftFrontDrive.power = (-yInput + xInput)* ONE_OVER_SQRT2 + rotationInput
        robot.rightRearDrive.power = (yInput - xInput)* ONE_OVER_SQRT2 + rotationInput
        robot.leftRearDrive.power = (-yInput - xInput)* ONE_OVER_SQRT2 + rotationInput
    }

    override fun stop() {
        with(robot) {
            stopAllMotors()
//            visualLocalizer.close()
            motionSensor.imu.stopAccelerationIntegration()
        }
    }
}