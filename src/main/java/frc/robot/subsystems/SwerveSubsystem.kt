package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

class SwerveModule(val driveMotor: CANSparkMax, val driveMotorConversionFactor: Double, val turnMotor: CANSparkMax, val turnMotorConversionFactor: Double, val turnAbsEncoder: AnalogInput) {
    init {
        driveMotor.encoder.positionConversionFactor = driveMotorConversionFactor
        turnMotor.encoder.positionConversionFactor = turnMotorConversionFactor
    }

    // Angle of turn motor in degrees
    var turnAngle: Double
        get() = (turnAbsEncoder.averageVoltage / 5.0) * 360.0
        set(position) = turnMotor.pidController.setReference(position, CANSparkMax.ControlType.kPosition).let { if (it != null) println("Failed to set module turn angle: $it") }

    // Velocity in centimeters/second
    var driveVelocity: Double
        get() = driveMotor.encoder.velocity
        set(speedCmPs) = driveMotor.pidController.setReference(speedCmPs, CANSparkMax.ControlType.kPosition).let { if (it != null) println("Failed to set module drive velocity: $it")}

    fun zeroTurnMotor() {
        /*
        turnMotor.set(0.05)
        var a: Double
        val w = 0.5

        while (true) {
            a = turnAngle
            if (equalInRange(a, 0.0, w) || equalInRange(a, 360.0, w)) break
        }
         */
        turnMotor.encoder.setPosition(0.0)
    }
}

fun zeroSwerveModules(modules: List<SwerveModule>) {
    for (module in modules) {
        module.turnMotor.set(0.05)
    }

    val w = 0.5
    while (true) {
        for (module in modules) {
            val a = module.turnAngle
            if (equalInRange(a, 0.0, w) || equalInRange(a, 360.0, w)) {
                module.turnMotor.stopMotor()
                module.zeroTurnMotor()
            }
        }
    }
}

fun equalInRange(input: Double, target: Double, wiggle: Double): Boolean {
    val lower = target - wiggle
    val higher = target + wiggle
    return (lower..higher).contains(input)
}

object SwerveSubsystem: SubsystemBase() {
    private val frontLeftLocation = Translation2d(Constants.MOTOR_PROPERTY_CONSTANTS.FRONT_LEFT_DX, Constants.MOTOR_PROPERTY_CONSTANTS.FRONT_LEFT_DY)
    private val frontRightLocation = Translation2d(Constants.MOTOR_PROPERTY_CONSTANTS.FRONT_RIGHT_DX, Constants.MOTOR_PROPERTY_CONSTANTS.FRONT_RIGHT_DY)
    private val rearLeftLocation = Translation2d(Constants.MOTOR_PROPERTY_CONSTANTS.REAR_LEFT_DX, Constants.MOTOR_PROPERTY_CONSTANTS.REAR_LEFT_DY)
    private val rearRightLocation = Translation2d(Constants.MOTOR_PROPERTY_CONSTANTS.REAR_RIGHT_DX, Constants.MOTOR_PROPERTY_CONSTANTS.REAR_RIGHT_DY)

    private val frontLeftMotor = SwerveModule(CANSparkMax(Constants.MOTOR_ID_CONSTANTS.FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless), Constants.MOTOR_PROPERTY_CONSTANTS.FRONT_LEFT_DRIVE_CONVERSION, CANSparkMax(Constants.MOTOR_ID_CONSTANTS.FRONT_LEFT_TURN_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless), Constants.MOTOR_PROPERTY_CONSTANTS.FRONT_LEFT_TURN_CONVERSION, AnalogInput(Constants.MOTOR_ID_CONSTANTS.FRONT_LEFT_TURN_ABS_ENCODER))
//    private val frontRightMotor = SwerveModule(CANSparkMax(Constants.MOTOR_ID_CONSTANTS.FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless), Constants.MOTOR_PROPERTY_CONSTANTS.FRONT_RIGHT_DRIVE_CONVERSION, CANSparkMax(Constants.MOTOR_ID_CONSTANTS.FRONT_RIGHT_TURN_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless), Constants.MOTOR_PROPERTY_CONSTANTS.FRONT_RIGHT_TURN_CONVERSION, AnalogInput(Constants.MOTOR_ID_CONSTANTS.FRONT_RIGHT_TURN_ABS_ENCODER))
//    private val rearLeftMotor = SwerveModule(CANSparkMax(Constants.MOTOR_ID_CONSTANTS.REAR_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless), Constants.MOTOR_PROPERTY_CONSTANTS.REAR_LEFT_DRIVE_CONVERSION, CANSparkMax(Constants.MOTOR_ID_CONSTANTS.REAR_LEFT_TURN_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless), Constants.MOTOR_PROPERTY_CONSTANTS.REAR_LEFT_TURN_CONVERSION, AnalogInput(Constants.MOTOR_ID_CONSTANTS.REAR_LEFT_TURN_ABS_ENCODER))
//    private val rearRightMotor = SwerveModule(CANSparkMax(Constants.MOTOR_ID_CONSTANTS.REAR_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless), Constants.MOTOR_PROPERTY_CONSTANTS.REAR_RIGHT_DRIVE_CONVERSION, CANSparkMax(Constants.MOTOR_ID_CONSTANTS.REAR_RIGHT_TURN_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless), Constants.MOTOR_PROPERTY_CONSTANTS.REAR_RIGHT_TURN_CONVERSION, AnalogInput(Constants.MOTOR_ID_CONSTANTS.REAR_RIGHT_TURN_ABS_ENCODER))

    // FL, FR, RL, RR
    private val kinematics = SwerveDriveKinematics(frontLeftLocation, frontRightLocation, rearLeftLocation, rearRightLocation)

    fun setFrontLeftAngle(angle: Double): CommandBase = runOnce {
        frontLeftMotor.turnAngle = angle
    }

    /**
     * This will set the motors to drive in the specified manner once
     *
     * **Important**
     * * `horizontalSpeed` and `verticalSpeed` must be in meters per second
     * * `angularRotation` must be in radians in usual Counter-Clockwise is positive form
     */
    fun driveSwerve(horizontalSpeed: Double, verticalSpeed: Double, angularRotation: Double): CommandBase = runOnce {
        val speed = ChassisSpeeds(horizontalSpeed, verticalSpeed, angularRotation)
        // FL, FR, RL, RR
        val states = kinematics.toSwerveModuleStates(speed)

        states[0] = states[0]!!.let {
            SwerveModuleState.optimize(it, Rotation2d.fromDegrees(frontLeftMotor.turnAngle))
        }
//        states[1] = states[0]!!.let {
//            SwerveModuleState.optimize(it, Rotation2d.fromDegrees(frontRightMotor.turnAngle))
//        }
//        states[2] = states[0]!!.let {
//            SwerveModuleState.optimize(it, Rotation2d.fromDegrees(rearLeftMotor.turnAngle))
//        }
//        states[3] = states[0]!!.let {
//            SwerveModuleState.optimize(it, Rotation2d.fromDegrees(rearRightMotor.turnAngle))
//        }

        states[0]!!.let {
            frontLeftMotor.turnAngle = it.angle.degrees
            frontLeftMotor.driveVelocity = it.speedMetersPerSecond * 100
        }
//        states[1]!!.let {
//            frontRightMotor.turnAngle = it.angle.degrees
//            frontRightMotor.driveVelocity = it.speedMetersPerSecond * 100
//        }
//        states[2]!!.let {
//            rearLeftMotor.turnAngle = it.angle.degrees
//            rearLeftMotor.driveVelocity = it.speedMetersPerSecond * 100
//        }
//        states[3]!!.let {
//            rearRightMotor.turnAngle = it.angle.degrees
//            rearRightMotor.driveVelocity = it.speedMetersPerSecond * 100
//        }
    }
}