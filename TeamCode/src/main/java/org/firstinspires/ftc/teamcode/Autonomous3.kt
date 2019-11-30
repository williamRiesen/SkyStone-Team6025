package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

lateinit var robot: TurtleDozer3

@Autonomous(name = "Autonomous3", group = "Holobot")
class Autonomous3 : LinearOpMode(){
    override fun runOpMode() {

        robot = TurtleDozer3(hardwareMap, telemetry)
        val startingXPosition = 24.0
        val startingYPosition = 60.0
        val driveFromStartToByFoundation = Vector(36,-24)
        robot.xPosition = startingXPosition
        robot.yPosition = startingYPosition
        robot.showStatus("Ready!")
        waitForStart()

        val moveToViewNavTarget = Vector(24,-12,name = "Move to View NavTarget")
        robot.driveByEncoder(moveToViewNavTarget)
        robot.updateSighting()
        val alignWithFoundation = Vector(48,36,name = "Align with foundation.")
        val backUpToLatchFoundation = Vector(48,24, name = "Back up to latch foundation.")
        robot.visuallyNavigateTo(alignWithFoundation)
        robot.deployHook()
        robot.visuallyNavigateTo(backUpToLatchFoundation)
        sleep(20000)
    }
}