package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.WheeeeelSubsystem;
import frc.robot.Constants.DriveConstants;

import java.util.function.DoubleSupplier;

/**
 * Hold driver RB to snap to and orbit the goal at a fixed radius.
 * Left X stick strafe walks the robot along the arc; robot always faces the goal.
 */
public class CommandOrbitGoal extends Command {

    private static final double kOrbitRadius = 2.0; // meters from goal center
    private static final double kRadialP     = 3.0; // tune if snap is too slow/aggressive
    private static final double kStickDeadband = 0.05;

    private final WheeeeelSubsystem m_drive;
    private final DoubleSupplier m_strafeInput; // driver left X axis (raw, -1 to 1)

    private final ProfiledPIDController m_headingPID = new ProfiledPIDController(
        5.0, 0.0, 0.0,
        new TrapezoidProfile.Constraints(
            DriveConstants.kMaxAngularSpeed,
            DriveConstants.kMaxAngularSpeed * 2.0));

    public CommandOrbitGoal(WheeeeelSubsystem drive, DoubleSupplier strafeInput) {
        m_drive = drive;
        m_strafeInput = strafeInput;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_headingPID.enableContinuousInput(-Math.PI, Math.PI);
        m_headingPID.reset(m_drive.getPose().getRotation().getRadians());
    }

    @Override
    public void execute() {
        Translation2d goalCenter = getGoalCenter();
        Translation2d robotPos   = m_drive.getPose().getTranslation();

        // Vector from goal to robot
        Translation2d toRobot = robotPos.minus(goalCenter);
        double currentRadius  = toRobot.getNorm();

        // Avoid divide-by-zero if robot is exactly on the goal center
        if (currentRadius < 0.01) return;

        double theta = Math.atan2(toRobot.getY(), toRobot.getX());

        // Radial correction: positive = away from goal, negative = toward goal
        double vRadial = kRadialP * (kOrbitRadius - currentRadius);
        // Clamp so we don't overshoot wildly
        vRadial = MathUtil.clamp(vRadial, -DriveConstants.kMaxSpeedMetersPerSecond,
                                           DriveConstants.kMaxSpeedMetersPerSecond);

        // Tangential velocity from stick (left X). Positive stick → CCW around goal.
        double stickX = MathUtil.applyDeadband(m_strafeInput.getAsDouble(), kStickDeadband);
        double vTangential = stickX * DriveConstants.kMaxSpeedMetersPerSecond;

        // Decompose into field-relative x/y
        // Radial unit:   ( cos θ,  sin θ )
        // Tangent unit:  (-sin θ,  cos θ )  (CCW)
        double vx = -vTangential * Math.sin(theta) + vRadial * Math.cos(theta);
        double vy =  vTangential * Math.cos(theta) + vRadial * Math.sin(theta);

        // Heading: always face the goal (angle from robot back to goal = theta + π)
        double targetHeading = MathUtil.angleModulus(theta + Math.PI);
        double rot = m_headingPID.calculate(
            m_drive.getPose().getRotation().getRadians(), targetHeading);
        rot = MathUtil.clamp(rot, -DriveConstants.kMaxAngularSpeed,
                                   DriveConstants.kMaxAngularSpeed);

        // Drive field-relative, no rate limiting (command owns the output)
        m_drive.drive(
            vx  / DriveConstants.kMaxSpeedMetersPerSecond,
            vy  / DriveConstants.kMaxSpeedMetersPerSecond,
            rot / DriveConstants.kMaxAngularSpeed,
            true,  // field relative
            false  // no slew rate limit
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true, false);
    }

    /** Returns the goal center for the current alliance. */
    private Translation2d getGoalCenter() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return new Translation2d(
                FieldConstants.Hub.oppTopCenterPoint.getX(),
                FieldConstants.Hub.oppTopCenterPoint.getY());
        }
        return new Translation2d(
            FieldConstants.Hub.topCenterPoint.getX(),
            FieldConstants.Hub.topCenterPoint.getY());
    }
}
