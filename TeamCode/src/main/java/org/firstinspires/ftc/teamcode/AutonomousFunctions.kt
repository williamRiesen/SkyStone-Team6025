package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.PI


lateinit var robot: TurtleDozerAutoBot3
val startLocation = useTileGrid(1.5,2.5)
var startHeading = PI / 2
var moveToViewNavTarget = useTileGrid(1.5,1.5)
val alignWithFoundation = Vector(48.0, 40.0, name = "Align with foundation.")
val backUpToLatchFoundation = Vector(48.0, 36.0, name = "Back up to latch foundation.")
val slideLeftToGoAroundFoundation = Vector(24.0, 60.0, name = "Slide Left to Go Around Foundation.")
val backUpAlongsideFoundation = Vector(24.0, 36.0, name = "Back up Alongside Foundation")
val pushFoundationToRight = Vector(36.0, 36.0, name = "Push Foundation to Right.")
val parkUnderBridge = Vector(0.0, 36.0, name = "Park Under Bridge.")
var foundationGoalLine = 60.0

fun initialize(hardwareMap: HardwareMap, telemetry: Telemetry, lightPattern: RevBlinkinLedDriver.BlinkinPattern) {
    telemetry.addData("Status","NOT READY! WAIT FOR SOLID LIGHT . . . ")
    telemetry.update()
    robot = TurtleDozerAutoBot3(hardwareMap, telemetry)
    robot.xPosition = startLocation.x
    robot.yPosition = startLocation.y
    robot.heading = robot.inertialMotionUnit.getHeading() + startHeading
    robot.blinkyLights.setPattern(lightPattern)
    robot.showStatus("Ready!")
}

fun useTileGrid(xTile: Double,yTile: Double): Vector{
    return Vector(xTile* 24.0, yTile * 24.0)
}
fun go(telemetry: Telemetry) {

    robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK)
    robot.navigateTo(moveToViewNavTarget)
//    robot.driveByEncoder(Vector(-0.0, -1.0* 24.0))
//    robot.navigateTo(moveToViewNavTarget)
//    robot.driveByEncoder(moveToViewNavTarget)
   robot.dozerBladePosition = 0.3
    robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE)
    Thread.sleep(10000)

//    robot.navigateTo(useTileGrid(1.0,1.0))
//    robot.updateSighting()
//    robot.navigateTo(alignWithFoundation)
//    robot.deployHook()
//    Thread.sleep(1000)
//    robot.navigateTo(backUpToLatchFoundation)
//    robot.dragWithVisualGuidance(foundationGoalLine)
//    robot.unlatchHook()
//    Thread.sleep(500)
//    robot.navigateTo(slideLeftToGoAroundFoundation)
//    robot.navigateTo(backUpAlongsideFoundation)
//    robot.navigateTo(pushFoundationToRight)
//    robot.navigateTo(parkUnderBridge)
//    robot.blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE
//  )
    robot.showStatus("Autonomous completed.")

}
