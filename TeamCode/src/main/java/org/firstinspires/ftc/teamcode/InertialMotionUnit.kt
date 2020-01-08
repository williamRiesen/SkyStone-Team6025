package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import kotlin.math.absoluteValue

class InertialMotionUnit(hardwareMap: HardwareMap) {

    private var imu: BNO055IMU
    private val parameters = Parameters()

    init {
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"
        parameters.accelerationIntegrationAlgorithm = com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator()

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get<BNO055IMU?>(BNO055IMU::
        class.java, "imu")!!
        imu.initialize(parameters)
    }

    fun getHeading(): Float {
        val angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS)
        return -angles.firstAngle
    }

    val xAccel
        get() = imu.linearAcceleration.xAccel

    val yAccel
        get() = imu.linearAcceleration.yAccel

    fun getAcceleration() = imu.linearAcceleration.xAccel.absoluteValue + imu.linearAcceleration.yAccel.absoluteValue
}
