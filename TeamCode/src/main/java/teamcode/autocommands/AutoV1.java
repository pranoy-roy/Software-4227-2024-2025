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
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.timer.TrcTimer;

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
        ARMMOVE,
        CLIP,
        MOVEBACK,
        OPENCLAWS,
        ARMMOVEBACK,
        MOVETOOBSERVATIONZONE,
        GRABSPECIMEN,
        PICKUPSPECIMEN,
        MOVETOCENTERWALL,
        PICKUPSAMPLE,
        ROTATEWITHSAMPLE,
        DROPSAMPLE,
        MOVEOUTOFOBSERVATIONZONE,
        MOVEINTOOBSERVATIONZONE,
        DONE
    }   //enum FirstState

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;

    private final TrcTimer firstTimer;
    private final TrcEvent firstEvent;
    private final TrcStateMachine<FirstState> firstSM;

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
        }
        else {
            robot.dashboard.displayPrintf(8, "State: " + firstState);
            robot.globalTracer.tracePreStateInfo(firstSM.toString(), firstState);
            switch (firstState) {
                case START:
                    if (autoChoices.delay > 0.0) {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + autoChoices.delay + "s.");
                        firstTimer.set(autoChoices.delay, firstEvent);
                        firstSM.waitForSingleEvent(firstEvent, FirstState.MOVE);
                    }
                    else {
                        robot.Lclaw.setLogicalPosition(0.5);
                        robot.Rclaw.setLogicalPosition(0);
                        robot.elbow.setPosition(-110);
                        robot.shoulder.setPosition(-75);
                        robot.robotDrive.driveBase.holonomicDrive(0.0, 0.5, 0.0, 0.25, firstEvent);
                        robot.robotDrive.driveBase.holonomicDrive(0.0, 0.0, 1.0, 0.375, firstEvent);
                        firstSM.waitForSingleEvent(firstEvent, FirstState.MOVE);
                    }
                    break;
                case MOVE:
                    robot.Lclaw.setLogicalPosition(0.5);
                    robot.Rclaw.setLogicalPosition(0);
                    robot.shoulder.setPosition(-75);
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 0.5, 0.0, 2.5, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.ARMMOVE);
                    break;
                case ARMMOVE:
                    robot.Lclaw.setLogicalPosition(0.5);
                    robot.Rclaw.setLogicalPosition(0);
                    robot.shoulder.setPosition(-75);
                    robot.elbow.setPosition(45);
                    firstTimer.set(1.5, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.CLIP);
                    break;
                case CLIP:
                    robot.Lclaw.setLogicalPosition(0.5);
                    robot.Rclaw.setLogicalPosition(0);
                    robot.shoulder.setPosition(-75);
                    robot.elbow.setPosition(15);
                    firstTimer.set(1.5, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.OPENCLAWS);
                    break;
                case OPENCLAWS:
                    robot.Lclaw.setLogicalPosition(0);
                    robot.Rclaw.setLogicalPosition(0.5);
                    robot.shoulder.setPosition(-75);
                    firstTimer.set(1.5, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.MOVEBACK);
                    break;
                case MOVEBACK:
                    robot.shoulder.setPosition(-75);
                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.5, 0.0, 1.5, firstEvent);
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 0.5, 0.0, 0.25, firstEvent);
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 0.0, 1.0, 0.75, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.ARMMOVEBACK);
                    break;
                case ARMMOVEBACK:
                    robot.elbow.setPosition(-110);
                    robot.shoulder.setPosition(-75);
                    firstTimer.set(2.0, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.MOVETOOBSERVATIONZONE);
                    break;
                case MOVETOOBSERVATIONZONE:
                    robot.shoulder.setPosition(-75);
                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.5, 0.0, 0.1, firstEvent);
                    robot.robotDrive.driveBase.holonomicDrive(0.5, 0.0, 0.0, 1.5, firstEvent);
                    if (!pickingUpSpecimen) {
                        pickingUpSpecimen = true;
                        firstSM.waitForSingleEvent(firstEvent, FirstState.GRABSPECIMEN);
                    }
                    /*else if (!pickingUpSample) {
                        pickingUpSample = true;
                        firstSM.waitForSingleEvent(firstEvent, FirstState.PICKUPSAMPLE);
                    }*/
                    else {
                        firstSM.waitForSingleEvent(firstEvent, FirstState.DONE);
                    }
                    break;
                case GRABSPECIMEN:
                    robot.elbow.setPosition(-100);
                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.5, 0.0, 0.1, firstEvent);
                    robot.Lclaw.setLogicalPosition(0.5);
                    robot.Rclaw.setLogicalPosition(0);
                    firstTimer.set(1.5, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.PICKUPSPECIMEN);
                    break;
                case PICKUPSPECIMEN:
                    robot.Lclaw.setLogicalPosition(0.5);
                    robot.Rclaw.setLogicalPosition(0);
                    robot.shoulder.setPosition(0);
                    firstTimer.set(1.5, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.MOVETOCENTERWALL);
                    break;
                case MOVETOCENTERWALL:
                    robot.Lclaw.setLogicalPosition(0.5);
                    robot.Rclaw.setLogicalPosition(0);
                    robot.robotDrive.driveBase.holonomicDrive(-0.5, 0.0, 0.0, 2.5, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.MOVE);
                    break;
                /*case PICKUPSAMPLE:
                    robot.robotDrive.driveBase.holonomicDrive(0.0,0.5,0.0,1.0, firstEvent);
                    robot.shoulder.setPosition(0);
                    robot.elbow.setPosition(15);
                    robot.Lclaw.setLogicalPosition(0.5);
                    robot.Rclaw.setLogicalPosition(0);
                    firstTimer.set(1.5, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.ROTATEWITHSAMPLE);
                    break;
                case ROTATEWITHSAMPLE:
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 0.0, 1.0, 0.8, firstEvent);
                    firstTimer.set(1.0, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.DROPSAMPLE);
                    break;
                case DROPSAMPLE:
                    robot.Lclaw.setLogicalPosition(0);
                    robot.Rclaw.setLogicalPosition(0.5);
                    firstTimer.set(1.0, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.MOVEOUTOFOBSERVATIONZONE);
                    break;
                case MOVEOUTOFOBSERVATIONZONE:
                    robot.elbow.setPosition(-110);
                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.5, 0.0, 1.0, firstEvent);
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 0.0, 1.0, 0.8, firstEvent);
                    firstTimer.set(4.0, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.MOVEINTOOBSERVATIONZONE);
                    break;
                case MOVEINTOOBSERVATIONZONE:
                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.5, 0.0, 1.0, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.GRABSPECIMEN);
                    break;*/
                case DONE:
                    // We are done.
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
