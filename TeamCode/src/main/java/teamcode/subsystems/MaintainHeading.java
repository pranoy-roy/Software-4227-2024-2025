/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.subsystems;

import teamcode.Robot;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcPidController;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

/**
 * This class creates the MaintainHeading subsystem of the Extender Arm.
 */
public class MaintainHeading {
    /**
     * This class contains MaintainHeading subsystem constants and parameters.
     */
    public static class Params {
        public static final TrcPidController.PidCoefficients posPidCoeffs =
                new TrcPidController.PidCoefficients(0.0275, 0.0, 0.002, 0.0, 0.0);
    }   //class Params

    private final Robot robot;
    public final TrcDbgTrace tracer;
    private TrcPidController headingPID;
    private double x, y;

    /**
     * Constructor: Creates an instance of the object.
     */
    public MaintainHeading(Robot robot, String instanceName) {
        this.robot = robot;
        this.tracer = new TrcDbgTrace();
        headingPID = new TrcPidController("headingPID", Params.posPidCoeffs, this::getHeading);
        headingPID.setInverted(true);
        headingPID.setTraceLevel(TrcDbgTrace.MsgLevel.DEBUG,true, null);
        TrcTaskMgr.TaskObject headingPidCtrlTaskObj = TrcTaskMgr.createTask(instanceName + ".pidCtrlTask", this::headingPidCtrlTask);
        headingPidCtrlTaskObj.registerTask(TrcTaskMgr.TaskType.OUTPUT_TASK);
    }   //MaintainHeading

    private double getHeading() {
        return robot.robotDrive.driveBase.getHeading();
    }

    public void setMaintainHeading(double myX, double myY, double myHeading) {
        x = myX;
        y = myY;
        headingPID.setAbsoluteSetPoint(true);
        headingPID.setTarget(myHeading);
        headingPID.setOutputLimit(0.5);
        headingPID.setNoOscillation(true);
        tracer.traceInfo("Heading Debug", "Heading: %f", myHeading);
    }

    private void headingPidCtrlTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop) {
        if (headingPID != null) {
            double angularVelocity = headingPID.getOutput();
            if (!headingPID.isOnTarget(1)) {
                robot.robotDrive.driveBase.holonomicDrive(x, y, angularVelocity);
            }
            else {
                robot.robotDrive.driveBase.holonomicDrive(x, y, 0);
            }
            tracer.traceInfo("Heading Debug", "X: %f, Y: %f, Angular Velocity: %f", x, y, angularVelocity);
        }
    }

    public void setTraceLevel(TrcDbgTrace.MsgLevel msgLevel) {
        tracer.setTraceLevel(msgLevel);
    }
}