package org.firstinspires.ftc.teamcode

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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor

@Autonomous(name = "Blue Autonomous", group = "Holobot")
@Disabled

class BlueAutonomous : LinearOpMode() {

    lateinit var robot: TurtleDozer

    val MOVE_NEXT_TO_FOUNDATION = Vector(25, -25)
    val ADVANCE_TO_LATCH_FOUNDATION = Vector(6,-2,0.1)
    val TOW_INTO_BUILDING_ZONE = Vector(0,50, 0.25)
    val MOVE_TO_PARKING_ZONE_UNDER_SKYBRIDGE= Vector (-48,0)

    override fun runOpMode() {

        robot = TurtleDozer.build(hardwareMap)

        waitForStart()
        robot.driveByEncoder(MOVE_NEXT_TO_FOUNDATION)
        telemetry.update()
        robot.deployHook()
        telemetry.update()
        robot.driveByEncoder(ADVANCE_TO_LATCH_FOUNDATION)
        robot.driveByEncoder(TOW_INTO_BUILDING_ZONE)
        robot.driveByGyro(TOW_INTO_BUILDING_ZONE,telemetry)
        robot.unlatchHook()
        sleep(500)
        robot.driveByEncoder(MOVE_TO_PARKING_ZONE_UNDER_SKYBRIDGE)

        while (opModeIsActive()) {
            sleep(50)
        }
        robot.stopAllMotors()
    }
}