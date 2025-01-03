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

package teamcode.autocommands;

import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.vision.Vision;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.timer.TrcTimer;
import trclib.pathdrive.TrcPose2D;
import teamcode.subsystems.PidDrive;
import trclib.vision.TrcOpenCvColorBlobPipeline;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements an autonomous strategy.
 */
public class AutoV1 implements TrcRobot.RobotCommand
{
    private final String moduleName = getClass().getSimpleName();

    private enum FirstState
    {
        START,
        MOVE,
        ROTATE,
        MOVETOCHAMBER,
        ARMMOVE,
        CLIP,
        OPENCLAWS,
        ARMMOVEBACK,
        MOVEBACK,
        MOVETOOBSERVATIONZONE,
        GRABSPECIMEN,
        PICKUPSPECIMEN,
        MOVETOCENTERWALL,
//        PICKUPSAMPLE,
//        ROTATEWITHSAMPLE,
//        DROPSAMPLE,
//        MOVEOUTOFOBSERVATIONZONE,
//        MOVEINTOOBSERVATIONZONE,
        DONE
    }   //enum FirstState

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer firstTimer;
    private final TrcEvent firstEvent;
    private final TrcStateMachine<FirstState> firstSM;
    private PidDrive pidDrive;
    private TrcPose2D pos0 = new TrcPose2D(0, 36, 270);
    private TrcPose2D pos1 = new TrcPose2D(0, 36, 0);
    private TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> sampleInfo;
    private boolean pickingUpSpecimen;
    //private boolean pickingUpSample;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public AutoV1(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        firstTimer = new TrcTimer(moduleName);
        firstEvent = new TrcEvent(moduleName);
        firstSM = new TrcStateMachine<>(moduleName);

        pidDrive = new PidDrive(robot);

        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE) {
            robot.vision.setSampleVisionEnabled(Vision.SampleType.RedSample, true);
        }
        else {
            robot.vision.setSampleVisionEnabled(Vision.SampleType.BlueSample, true);
        }

        pickingUpSpecimen = false;
        //pickingUpSample = false;

        firstSM.start(FirstState.START);
    }   //CmdAuto

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return firstSM.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        firstTimer.cancel();
        firstSM.stop();
    }   //cancel



    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        FirstState firstState = firstSM.checkReadyAndGetState();

        if (firstState == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=" + firstSM.getNextState() + ")...");
            robot.dashboard.displayPrintf(9, "Robot Pose: (%f, %f, %f)",
                    robot.robotDrive.driveBase.getXPosition(),
                    robot.robotDrive.driveBase.getYPosition(),
                    robot.robotDrive.driveBase.getHeading());
        }
        else {
            robot.dashboard.displayPrintf(8, "State: " + firstState);
            robot.globalTracer.tracePreStateInfo(firstSM.toString(), firstState);
            robot.dashboard.displayPrintf(9, "Robot Pose: (%f, %f, %f)",
                    robot.robotDrive.driveBase.getXPosition(),
                    robot.robotDrive.driveBase.getYPosition(),
                    robot.robotDrive.driveBase.getHeading());
            switch (firstState) {
                case START:
                    if (autoChoices.delay > 0.0) {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + autoChoices.delay + "s.");
                        firstTimer.set(autoChoices.delay, firstEvent);
                        firstSM.waitForSingleEvent(firstEvent, FirstState.MOVE);
                    }
                    else {
                        robot.LClaw.setLogicalPosition(0.5);
                        robot.RClaw.setLogicalPosition(0);
                        robot.elbow.setPosition(-130.86);
                        robot.shoulder.setPosition(-83.6);
                        firstSM.setState(FirstState.MOVE);
                    }
                    break;
                case MOVE:
                    robot.LClaw.setLogicalPosition(0.5);
                    robot.RClaw.setLogicalPosition(0);
                    robot.elbow.setPosition(-130.86);
                    robot.shoulder.setPosition(-83.6);
                    pidDrive.setPidDrive(new TrcPose2D(-2, 0, 0), firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.ROTATE);
                    break;
                case ROTATE:
                    robot.LClaw.setLogicalPosition(0.5);
                    robot.RClaw.setLogicalPosition(0);
                    robot.elbow.setPosition(-130.86);
                    robot.shoulder.setPosition(-83.6);
                    pickingUpSpecimen = true;
                    pidDrive.setPidDrive(new TrcPose2D(-2, 0, -90), firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.MOVETOCHAMBER);
                    break;
                case MOVETOCHAMBER:
                    robot.LClaw.setLogicalPosition(0.5);
                    robot.RClaw.setLogicalPosition(0);
                    robot.elbow.setPosition(-130.86);
                    robot.shoulder.setPosition(-83.6);
                    pidDrive.resetPosition();
                    pidDrive.setPidDrive(new TrcPose2D(0, 29, 0), firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.ARMMOVE);
                    break;
                case ARMMOVE:
                    robot.LClaw.setLogicalPosition(0.5);
                    robot.RClaw.setLogicalPosition(0);
                    robot.elbow.setPosition(20.13);
                    robot.shoulder.setPosition(-83.6);
                    firstTimer.set(2.0, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.CLIP);
                    break;
                case CLIP:
                    robot.LClaw.setLogicalPosition(0.5);
                    robot.RClaw.setLogicalPosition(0);
                    robot.elbow.setPosition(20.13);
                    robot.shoulder.setPosition(-83.6);
                    pidDrive.setPidDrive(new TrcPose2D(0, 32, 0), firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.OPENCLAWS);
                    break;
                case OPENCLAWS:
                    robot.LClaw.setLogicalPosition(0);
                    robot.RClaw.setLogicalPosition(0.5);
                    robot.elbow.setPosition(20.13);
                    robot.shoulder.setPosition(-83.6);
                    firstTimer.set(1.0, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.ARMMOVEBACK);
                    break;
                case ARMMOVEBACK:
                    robot.elbow.setPosition(-130.86);
                    robot.shoulder.setPosition(-83.6);
                    firstTimer.set(2.0, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.MOVEBACK);
                    break;
                case MOVEBACK:
                    robot.elbow.setPosition(-130.86);
                    robot.shoulder.setPosition(-83.6);
                    pidDrive.setPidDrive(new TrcPose2D(0, 2, 0), firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.MOVETOOBSERVATIONZONE);
                    break;
                case MOVETOOBSERVATIONZONE:
                    robot.elbow.setPosition(-130.86);
                    robot.shoulder.setPosition(-83.6);
                    pidDrive.setPidDrive(new TrcPose2D(48, 2, 0), firstEvent);
                    if (!pickingUpSpecimen) {
                        firstSM.waitForSingleEvent(firstEvent, FirstState.DONE);
                    }
                    else {
                        firstSM.waitForSingleEvent(firstEvent, FirstState.GRABSPECIMEN);
                    }
                    break;
                case GRABSPECIMEN:
                    robot.elbow.setPosition(-130.86);
                    robot.shoulder.setPosition(-83.6);
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE) {
                        sampleInfo = robot.vision.getDetectedSample(Vision.SampleType.RedSample, 0.0, 10);
                    }
                    else {
                        sampleInfo = robot.vision.getDetectedSample(Vision.SampleType.BlueSample, 0.0, 10);
                    }

                    if (sampleInfo != null) {
                        if (sampleInfo.detectedObj.getObjectArea() >= 150000) {
                            robot.LClaw.setLogicalPosition(0.5);
                            robot.RClaw.setLogicalPosition(0);
                            firstTimer.set(1.0, firstEvent);
                            firstSM.waitForSingleEvent(firstEvent, FirstState.PICKUPSPECIMEN);
                        }
                        else {
                            firstSM.setState(FirstState.DONE);
                        }
                    }
                    break;
                case PICKUPSPECIMEN:
                    robot.LClaw.setLogicalPosition(0.5);
                    robot.RClaw.setLogicalPosition(0);
                    robot.elbow.setPosition(-130.86);
                    robot.shoulder.setPosition(0);
                    pickingUpSpecimen = false;
                    firstTimer.set(2.0, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.MOVETOCENTERWALL);
                    break;
                case MOVETOCENTERWALL:
                    robot.LClaw.setLogicalPosition(0.5);
                    robot.RClaw.setLogicalPosition(0);
                    robot.elbow.setPosition(-130.86);
                    robot.shoulder.setPosition(-83.6);
                    pidDrive.setPidDrive(new TrcPose2D(0, 2, 0), firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.MOVETOCHAMBER);
                    break;
                case DONE:
                    cancel();
                    break;
                default:
            }

            robot.globalTracer.tracePostStateInfo(
                    firstSM.toString(), firstState, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                    robot.robotDrive.purePursuitDrive, null);
        }

        return !firstSM.isEnabled();
    }   //cmdPeriodic

}   //class CmdAuto
