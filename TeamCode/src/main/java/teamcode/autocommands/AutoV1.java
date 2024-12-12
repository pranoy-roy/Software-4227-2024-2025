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
        MOVEBACK,
        OPENCLAWS,
        ARMMOVEBACK,
        MOVETOOBSERVATIONZONE,
        DONE
    }

    private enum SecondState
    {
        START,
        CLOSECLAWS,
        MOVEARMBASEBACK,
        DONE
    }

    private enum ThirdState
    {
        START,
        MOVEFORWARD,
        LOWERARM,
        CLOSECLAWS,
        RAISEARM,
        MOVEFORWARDTOSPIKE,
        ROTATE,
        MOVEINTOOBSERVATIONZONE,
        LOWERARMWITHSAMPLE,
        OPENCLAWS,
        RAISEARMBASEANDARM,
        MOVEOUTOFOBSERVATIONZONE,
        ROTATEBACK,
        WAIT,
        MOVEBACKINTOOBSERVATIONZONE,
        DONE
    }

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;

    private final TrcTimer firstTimer;
    private final TrcEvent firstEvent;
    private final TrcStateMachine<FirstState> firstSM;

    private final TrcTimer secondTimer;
    private final TrcEvent secondEvent;
    private final TrcStateMachine<SecondState> secondSM;

    private final TrcTimer thirdTimer;
    private final TrcEvent thirdEvent;
    private final TrcStateMachine<ThirdState> thirdSM;

    private boolean finishedFirstState;
    private boolean finishedSecondState;
    private boolean finishedThirdState;

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

        secondTimer = new TrcTimer(moduleName);
        secondEvent = new TrcEvent(moduleName);
        secondSM = new TrcStateMachine<>(moduleName);

        thirdTimer = new TrcTimer(moduleName);
        thirdEvent = new TrcEvent(moduleName);
        thirdSM = new TrcStateMachine<>(moduleName);

        finishedFirstState = true;
        finishedSecondState = false;
        finishedThirdState = false;

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
        return firstSM.isEnabled() && secondSM.isEnabled() && thirdSM.isEnabled();
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

    public void secondCancel()
    {
        secondTimer.cancel();
        secondSM.stop();
    }   //cancel

    public void thirdCancel()
    {
        thirdTimer.cancel();
        thirdSM.stop();
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
        SecondState secondState = secondSM.checkReadyAndGetState();
        ThirdState thirdState = thirdSM.checkReadyAndGetState();

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
                        firstSM.setState(FirstState.MOVE);
                    }
                    break;
                case MOVE:
                    robot.armBase.setMotorPower(1);
                    if (finishedSecondState) {
                        robot.robotDrive.driveBase.holonomicDrive(-0.5, 0.0, 0.0, 1.0, firstEvent);
                    }
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 0.5, 0.0, 0.5, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.ARMMOVE);
                    break;
                case ARMMOVE:
                    robot.armBase.setMotorPower(1);
                    robot.arm.setMotorPower(0.5);
                    firstTimer.set(2.5, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.OPENCLAWS);
                    break;
                case OPENCLAWS:
                    robot.armBase.setMotorPower(1);
                    robot.Lclaw.setLogicalPosition(0);
                    robot.Rclaw.setLogicalPosition(0.5);
                    firstTimer.set(1.0, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.ARMMOVEBACK);
                    break;
                case ARMMOVEBACK:
                    robot.armBase.setMotorPower(1);
                    robot.arm.setMotorPower(-0.5);
                    firstTimer.set(1.0, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.MOVEBACK);
                    break;
                case MOVEBACK:
                    robot.armBase.setMotorPower(1);
                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.5, 0.0, 0.5, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.MOVETOOBSERVATIONZONE);
                    break;
                case MOVETOOBSERVATIONZONE:
                    robot.armBase.setMotorPower(1);
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 0.5, 0.0, 0.2, firstEvent);
                    robot.robotDrive.driveBase.holonomicDrive(0.5, 0.0, 0.0, 1.0, firstEvent);
                    firstSM.waitForSingleEvent(firstEvent, FirstState.DONE);
                    break;
                case DONE:
                    if (finishedFirstState) {
                        secondSM.setState(SecondState.START);
                        finishedFirstState = false;
                    }

                    else if (finishedSecondState && !finishedThirdState) {
                        thirdSM.setState(ThirdState.START);
                        finishedSecondState = false;
                    }

                    else {
                        cancel();
                    }
                    break;

                default:
            }

            robot.globalTracer.tracePostStateInfo(
                    firstSM.toString(), firstState, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                    robot.robotDrive.purePursuitDrive, null);
        }

        if (secondState == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=" + secondSM.getNextState() + ")...");
        }
        else {
            robot.dashboard.displayPrintf(8, "State: " + secondState);
            robot.globalTracer.tracePreStateInfo(secondSM.toString(), secondState);
            switch (secondState) {
                case START:
                    secondSM.setState(SecondState.CLOSECLAWS);
                    break;
                case CLOSECLAWS:
                    robot.armBase.setMotorPower(1);
                    robot.Lclaw.setLogicalPosition(0.75);
                    robot.Rclaw.setLogicalPosition(0);
                    robot.Lclaw.setControllerOn(true);
                    robot.Rclaw.setControllerOn(true);
                    secondTimer.set(1.0, secondEvent);
                    secondSM.waitForSingleEvent(secondEvent, SecondState.MOVEARMBASEBACK);
                    break;
                case MOVEARMBASEBACK:
                    robot.armBase.setMotorPower(-0.5);
                    secondTimer.set(0.5, secondEvent);
                    secondSM.waitForSingleEvent(secondEvent, SecondState.DONE);
                    break;
                case DONE:
                    finishedSecondState = true;
                    firstSM.setState(FirstState.START);
                    secondCancel();
                    break;

                default:
            }

            robot.globalTracer.tracePostStateInfo(
                    secondSM.toString(), secondState, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                    robot.robotDrive.purePursuitDrive, null);
        }

        if (thirdState == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=" + thirdSM.getNextState() + ")...");
        }
        else {
            robot.dashboard.displayPrintf(8, "State: " + thirdState);
            robot.globalTracer.tracePreStateInfo(thirdSM.toString(), thirdState);
            switch (thirdState) {
                case START:
                    thirdSM.setState(ThirdState.MOVEFORWARD);
                    break;
                case MOVEFORWARD:
                    robot.armBase.setMotorPower(1);
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 0.5, 0.0, 0.2, thirdEvent);
                    thirdSM.waitForSingleEvent(thirdEvent, ThirdState.LOWERARM);
                    break;
                case LOWERARM:
                    robot.arm.setMotorPower(1);
                    thirdTimer.set(1.5, thirdEvent);
                    thirdSM.waitForSingleEvent(thirdEvent, ThirdState.CLOSECLAWS);
                    break;
                case CLOSECLAWS:
                    robot.Lclaw.setLogicalPosition(0.75);
                    robot.Rclaw.setLogicalPosition(0);
                    robot.Lclaw.setControllerOn(true);
                    robot.Rclaw.setControllerOn(true);
                    thirdTimer.set(1.0, thirdEvent);
                    thirdSM.waitForSingleEvent(thirdEvent, ThirdState.RAISEARM);
                    break;
                case RAISEARM:
                    robot.arm.setMotorPower(-1);
                    thirdTimer.set(1.5, thirdEvent);
                    thirdSM.waitForSingleEvent(thirdEvent, ThirdState.MOVEFORWARDTOSPIKE);
                    break;
                case MOVEFORWARDTOSPIKE:
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 0.5, 0.0, 0.15, thirdEvent);
                    thirdSM.waitForSingleEvent(thirdEvent, ThirdState.ROTATE);
                    break;
                case ROTATE:
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 0.0, 1.0, 2.0, thirdEvent);
                    thirdSM.waitForSingleEvent(thirdEvent, ThirdState.MOVEINTOOBSERVATIONZONE);
                    break;
                case MOVEINTOOBSERVATIONZONE:
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 0.5, 0.0, 0.3, thirdEvent);
                    thirdSM.waitForSingleEvent(thirdEvent, ThirdState.LOWERARMWITHSAMPLE);
                    break;
                case LOWERARMWITHSAMPLE:
                    robot.arm.setMotorPower(1);
                    thirdTimer.set(1.5, thirdEvent);
                    thirdSM.waitForSingleEvent(thirdEvent, ThirdState.OPENCLAWS);
                    break;
                case OPENCLAWS:
                    robot.Lclaw.setLogicalPosition(0);
                    robot.Rclaw.setLogicalPosition(0.5);
                    thirdTimer.set(1.0, thirdEvent);
                    thirdSM.waitForSingleEvent(thirdEvent, ThirdState.RAISEARMBASEANDARM);
                    break;
                case RAISEARMBASEANDARM:
                    robot.armBase.setMotorPower(1);
                    robot.arm.setMotorPower(-1);
                    thirdTimer.set(2.0, thirdEvent);
                    thirdSM.waitForSingleEvent(thirdEvent, ThirdState.MOVEOUTOFOBSERVATIONZONE);
                    break;
                case MOVEOUTOFOBSERVATIONZONE:
                    robot.armBase.setMotorPower(1);
                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.5, 0.0, 0.2, thirdEvent);
                    thirdSM.waitForSingleEvent(thirdEvent, ThirdState.ROTATEBACK);
                    break;
                case ROTATEBACK:
                    robot.armBase.setMotorPower(1);
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 0.0, -1.0, 2.0, thirdEvent);
                    thirdSM.waitForSingleEvent(thirdEvent, ThirdState.WAIT);
                    break;
                case WAIT:
                    robot.armBase.setMotorPower(1);
                    thirdTimer.set(4.0, thirdEvent);
                    thirdSM.waitForSingleEvent(thirdEvent, ThirdState.MOVEBACKINTOOBSERVATIONZONE);
                    break;
                case MOVEBACKINTOOBSERVATIONZONE:
                    robot.armBase.setMotorPower(1);
                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.5, 0.0, 0.2, thirdEvent);
                    thirdSM.waitForSingleEvent(thirdEvent, ThirdState.DONE);
                    break;
                case DONE:
                    finishedThirdState = true;
                    secondSM.setState(SecondState.START);
                    thirdCancel();
                    break;

                default:
            }

            robot.globalTracer.tracePostStateInfo(
                    thirdSM.toString(), thirdState, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                    robot.robotDrive.purePursuitDrive, null);
        }

        return !firstSM.isEnabled() && !secondSM.isEnabled() && !thirdSM.isEnabled();
    }   //cmdPeriodic

}   //class CmdAuto
