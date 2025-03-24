package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

    public static final class Constants{
        public static final int MAX_SPEED = 0;
    }

    /** Swerve drive object */
    private final SwerveDrive swerveDrive;

    public SwerveSubsystem(File directory) {
        boolean blueAlliance = false;
        Pose2d startingPose = blueAlliance
                ? new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromRadians(0))
                : new Pose2d(new Translation2d(Meter.of(16), Meter.of(4)), Rotation2d.fromRadians(Math.PI));
        
        //Configure telemetry 
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, startingPose);
        }catch (Exception e){
            throw new RuntimeException(e);
        }

        //according to YAGSL example, should only be true if controlling robot by angle
        swerveDrive.setHeadingCorrection(false);

        //!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);

        //docs say to start with comp value of 0.1
        swerveDrive.setAngularVelocityCompensation(
            true, 
            true, 
            0.1);

        //enable to resynchronize absolute and relative encoders when not moving for a few seconds
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

        //setupPathPlanner() - once implemented
        //RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance)); - once zeroGyroWithAlliance implemented

    }
}
