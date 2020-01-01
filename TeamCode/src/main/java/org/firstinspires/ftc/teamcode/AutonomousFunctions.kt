package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.PI


lateinit var robot: TurtleDozerAutoBot3
val startLocation = useTileGrid(0.0, 0.0)
var startHeading = PI / 2
//var moveToViewNavTarget = useTileGrid(0.5, -1.0)
val alignWithFoundation = useTileGrid(0.5, -1.0)
val bumpAudienceWall = useTileGrid(2.0,0.0)
val reboundToAlignWithFoundation = AutonomousStep(-16.0,0.0,0.5)
val backUpToLatchFoundation = useTileGrid(0.0, -0.35, 0.1)
val pullForwardToLatch = useTileGrid(0.0, 0.35, 0.1)
val dragFoundation = useTileGrid(0.0, 1.5, 0.3)
val slideLeftToGoAroundFoundation = useTileGrid(1.0, 2.5)
val shiftTowardAudience = useTileGrid(-1.25, 0.0)
val backUpToGoAroundFoundation = useTileGrid(0.0, -1.75)
val advanceAwayFromAudience = useTileGrid(0.0, 1.25)
val pushFoundationHome = useTileGrid(-2.0,0.0,0.5)
val backUpAlongsideFoundation = useTileGrid(1.0, 2.0)
val pushFoundationToRight = useTileGrid(1.5, 2.0)
val parkUnderBridge = useTileGrid(0.0,-2.0)
val goForwardOneTile = useTileGrid(1.0, 0.0, 0.5, "Go Forward One Tile.")
var foundationGoalLine = 60.0
val pointA = useTileGrid(1.5, 2.6)

fun initialize(hardwareMap: HardwareMap, telemetry: Telemetry, lightPattern: RevBlinkinLedDriver.BlinkinPattern) {
    telemetry.addData("Status", "NOT READY! WAIT FOR SOLID LIGHT . . . ")
    telemetry.update()
    robot = TurtleDozerAutoBot3(hardwareMap, telemetry)
    with(robot) {
        xPosition = startLocation.x
        yPosition = startLocation.y
//        heading = inertialMotionUnit.getHeading() + startHeading
        blinkyLights.setPattern(lightPattern)
        showStatus("Ready!")
        kennethClawLeft.direction = Servo.Direction.REVERSE
    }
}

fun useTileGrid(xTile: Double, yTile: Double, speed: Double = 0.75, name: String = "Unnamed Instruction Step"): AutonomousStep {
    return AutonomousStep(xTile * 24.0, yTile * 24.0, speed, name)
}

fun go(telemetry: Telemetry) {
    with(robot) {
        desiredHeading = PI / 2.0
        blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK)
        drive(alignWithFoundation)
        bumpDrive(bumpAudienceWall)
        drive(reboundToAlignWithFoundation)
        deployHook()
        drive(backUpToLatchFoundation)
        drive(dragFoundation)
        unlatchHook()
        drive(shiftTowardAudience)
        drive(backUpToGoAroundFoundation)
        rotateToHeading(PI)
        drive(advanceAwayFromAudience)
        drive(pushFoundationHome)
        drive(parkUnderBridge)
        kennethClawLeft.position = clawRestPosition
        kennethClawRight.position = clawRestPosition
        blinkyLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE)
    }
    Thread.sleep(9000)
}
