package frc.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.PlaceholderSubsystem.runOnce
import frc.robot.subsystems.SwerveSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer
{
    init
    {
        SmartDashboard.putNumber("Front Left Angle", 0.0)
        configureBindings()
    }

    private val driverController = CommandXboxController(Constants.DRIVER_CONSTANTS.DRIVER_CONTROLLER_PORT)

    /** Use this method to define your `trigger->command` mappings. */
    private fun configureBindings()
    {
        driverController.leftTrigger().onTrue(FrontLeftAngleCommand())
    }

    fun getAutonomousCommand(): Command?
    {
        // TODO: Implement properly
        return null
    }
}

class FrontLeftAngleCommand: CommandBase() {
    init {
        addRequirements(SwerveSubsystem)
    }

    override fun initialize() {
        SwerveSubsystem.setFrontLeftAngle(SmartDashboard.getNumber("Front Left Angle", 0.0))
    }

    override fun isFinished(): Boolean {
        return true
    }
}