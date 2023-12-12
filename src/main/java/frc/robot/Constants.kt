package frc.robot

object Constants {
    object DRIVER_CONSTANTS {
        const val DRIVER_CONTROLLER_PORT = 0
        const val MANIPULATOR_CONTROLLER_PORT = 1
    }

    object MOTOR_ID_CONSTANTS {
        const val FRONT_LEFT_DRIVE_MOTOR = 10
        const val FRONT_LEFT_TURN_MOTOR = 11
        const val FRONT_LEFT_TURN_ABS_ENCODER = 0

        const val FRONT_RIGHT_DRIVE_MOTOR = 12
        const val FRONT_RIGHT_TURN_MOTOR = 13
        const val FRONT_RIGHT_TURN_ABS_ENCODER = 1

        const val REAR_LEFT_DRIVE_MOTOR = 14
        const val REAR_LEFT_TURN_MOTOR = 15
        const val REAR_LEFT_TURN_ABS_ENCODER = 2

        const val REAR_RIGHT_DRIVE_MOTOR = 16
        const val REAR_RIGHT_TURN_MOTOR = 17
        const val REAR_RIGHT_TURN_ABS_ENCODER = 3
    }

    object MOTOR_PROPERTY_CONSTANTS {
        const val FRONT_LEFT_DRIVE_CONVERSION = 1.0
        const val FRONT_LEFT_TURN_CONVERSION = 1.0

        const val FRONT_RIGHT_DRIVE_CONVERSION = 1.0
        const val FRONT_RIGHT_TURN_CONVERSION = 1.0

        const val REAR_LEFT_DRIVE_CONVERSION = 1.0
        const val REAR_LEFT_TURN_CONVERSION = 1.0

        const val REAR_RIGHT_DRIVE_CONVERSION = 1.0
        const val REAR_RIGHT_TURN_CONVERSION = 1.0

        const val FRONT_LEFT_DX = -0.4572
        const val FRONT_LEFT_DY = 0.3048

        const val FRONT_RIGHT_DX = 0.4572
        const val FRONT_RIGHT_DY = 0.3048

        const val REAR_LEFT_DX = -0.4572
        const val REAR_LEFT_DY = -0.3048

        const val REAR_RIGHT_DX = 0.4572
        const val REAR_RIGHT_DY = -0.3048
    }
}