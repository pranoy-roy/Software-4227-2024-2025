
package teamcode.subsystems;

import teamcode.Robot;
import trclib.pathdrive.TrcPidDrive;
import trclib.robotcore.TrcPidController;

public class PidDriveParams {
    private final Robot robot;
    private TrcPidDrive pidDrive;
    private TrcPidController.PidCoefficients xPidCoeffs = new
            TrcPidController.PidCoefficients(0.5,0.0, 0.002, 0.0, 0.0);
    private TrcPidController.PidCoefficients yPidCoeffs = new
            TrcPidController.PidCoefficients(0.5,0.0, 0.002, 0.0, 0.0);
    private TrcPidController.PidCoefficients turnPidCoeffs = new
            TrcPidController.PidCoefficients(0.5,0.0, 0.002, 0.0, 0.0);
    public PidDriveParams(Robot robot) {
        this.robot = robot;
        pidDrive = new TrcPidDrive("pidDrive", robot.robotDrive.driveBase,
                xPidCoeffs, 1.0, this::getX,
                yPidCoeffs, 1.0, this::getY,
                turnPidCoeffs, 1.0, this::getAngle);
    }
    public TrcPidDrive getPidDrive() {
        return pidDrive;
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
