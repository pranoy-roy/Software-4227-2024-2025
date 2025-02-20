
package teamcode.subsystems;

import teamcode.Robot;
import trclib.pathdrive.TrcPidDrive;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcPidController;

public class PidDrive {
    private final Robot robot;
    private TrcPidDrive pidDrive;
    private TrcPidController.PidCoefficients xPidCoeffs = new
            TrcPidController.PidCoefficients(0.05, 0.0, 0.002, 0.0, 0.0);
    private TrcPidController.PidCoefficients yPidCoeffs = new
            TrcPidController.PidCoefficients(0.05, 0.0, 0.002, 0.0, 0.0);
    private TrcPidController.PidCoefficients turnPidCoeffs = new
            TrcPidController.PidCoefficients(0.0275, 0.0, 0.002, 0.0, 0.0);
    public PidDrive(Robot robot) {
        this.robot = robot;
        pidDrive = new TrcPidDrive("pidDrive", robot.robotDrive.driveBase,
                xPidCoeffs, 0.25, this::getX,
                yPidCoeffs, 0.25, this::getY,
                turnPidCoeffs, 0.5, this::getAngle);

        pidDrive.setNoOscillation(true);
        pidDrive.setAbsolutePose(new TrcPose2D(0, 0, 0));

        pidDrive.getXPidCtrl().setOutputLimit(0.8);
        pidDrive.getYPidCtrl().setOutputLimit(0.8);
        pidDrive.getTurnPidCtrl().setOutputLimit(0.8);

        pidDrive.getTurnPidCtrl().setInverted(true);
        pidDrive.getTurnPidCtrl().setAbsoluteSetPoint(true);

        pidDrive.setTraceLevel(TrcDbgTrace.MsgLevel.DEBUG, true, true, true);
    }
    public void setPidDrive(TrcPose2D pose, TrcEvent event) {
        pose.x *= 1.31 * Math.pow(0.995479, ((pose.x/12) - 1));
        pose.y *= 1.25 * Math.pow(0.929782, ((pose.y/12) - 1));
        pidDrive.setAbsoluteTarget(pose, false, event);
    }

    private double getX() {
        return robot.robotDrive.driveBase.getXPosition();
    }
    private double getY() {
        return robot.robotDrive.driveBase.getYPosition();
    }
    private double getAngle() {
        return robot.robotDrive.driveBase.getHeading();
    }
}
