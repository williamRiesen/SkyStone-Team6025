package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.*

class SimulatedDcMotor : DcMotor {
    override fun setMotorType(motorType: MotorConfigurationType?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun resetDeviceConfigurationForOpMode() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getController(): DcMotorController {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getDeviceName(): String {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getCurrentPosition(): Int {
        return 9
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setTargetPosition(position: Int) {
    }


    override fun getPowerFloat(): Boolean {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getConnectionInfo(): String {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getVersion(): Int {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getMode(): DcMotor.RunMode {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getPower(): Double {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getPortNumber(): Int {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun isBusy(): Boolean {
        return false
    }

    override fun close() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setPowerFloat() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setPower(power: Double) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setMode(mode: DcMotor.RunMode?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setDirection(direction: DcMotorSimple.Direction?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getMotorType(): MotorConfigurationType {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getTargetPosition(): Int {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getDirection(): DcMotorSimple.Direction {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getZeroPowerBehavior(): DcMotor.ZeroPowerBehavior {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
}



class SimulatedServo : Servo {
    override fun resetDeviceConfigurationForOpMode() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setDirection(direction: Servo.Direction?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getController(): ServoController {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getPosition(): Double {
        return position
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getDeviceName(): String {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun scaleRange(min: Double, max: Double) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setPosition(position: Double) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getConnectionInfo(): String {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getVersion(): Int {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getDirection(): Servo.Direction {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getPortNumber(): Int {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun close() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
}
class SimulatedIMU:BNO055IMU {
    override fun initialize(parameters: BNO055IMU.Parameters): Boolean {
        return true
    }
    override fun getOverallAcceleration(): Acceleration {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun getQuaternionOrientation(): Quaternion {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun getCalibrationStatus(): BNO055IMU.CalibrationStatus {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun getVelocity(): Velocity {
        return Velocity(DistanceUnit.CM, 1.0,1.0,1.0,1)
    }
    override fun write8(register: BNO055IMU.Register?, bVal: Int) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun getPosition(): Position {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
        return Position(DistanceUnit.CM, 1.0, 1.0, 1.0, 1)
    }
    override fun getTemperature(): Temperature {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun getGravity(): Acceleration {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun getParameters(): BNO055IMU.Parameters {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun getSystemStatus(): BNO055IMU.SystemStatus {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun getLinearAcceleration(): Acceleration {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun getAcceleration(): Acceleration {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun isAccelerometerCalibrated(): Boolean {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun isMagnetometerCalibrated(): Boolean {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun getMagneticFieldStrength(): MagneticFlux {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun getAngularOrientation(reference: AxesReference?, order: AxesOrder?, angleUnit: AngleUnit?): Orientation {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun getAngularOrientation(): Orientation {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun getAngularVelocity(): AngularVelocity {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun startAccelerationIntegration(initialPosition: Position?, initialVelocity: Velocity?, msPollInterval: Int) {
//         //To change body of created functions use File | Settings | File Templates.
    }
    override fun isSystemCalibrated(): Boolean {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun stopAccelerationIntegration() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun writeCalibrationData(data: BNO055IMU.CalibrationData?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun isGyroCalibrated(): Boolean {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun write(register: BNO055IMU.Register?, data: ByteArray?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun read(register: BNO055IMU.Register?, cb: Int): ByteArray {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun readCalibrationData(): BNO055IMU.CalibrationData {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun getSystemError(): BNO055IMU.SystemError {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun read8(register: BNO055IMU.Register?): Byte {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun close() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
}


class SimulatedMotionSensor: MotionSensor {
    val imu = SimulatedIMU()
    val parameters = BNO055IMU.Parameters()
    init {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"
        parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()
//        imu = hwMap.get(BNO055IMU::class.java, "imu")
        imu.initialize(parameters)
        imu.startAccelerationIntegration(Position(), Velocity(), 1000)
    }
    override fun getHeading(): Float {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun resetHeading() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

}


