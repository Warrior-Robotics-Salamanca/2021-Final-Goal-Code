package org.firstinspires.ftc.teamcode.GameOpModes.CompetitionOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.LaunchRings;
import org.firstinspires.ftc.teamcode.commands.RunnableCommand;
import org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.TargetPose;
import org.firstinspires.ftc.teamcode.constants.DriveBaseConstants;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.constants.LauncherConstants;
import org.firstinspires.ftc.teamcode.localizers.DeadWheelsAndIMULocalizer;
import org.firstinspires.ftc.teamcode.localizers.SwerveUnitAndIMULocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;
import org.firstinspires.ftc.teamcode.util.Button;
import org.firstinspires.ftc.teamcode.util.InterOpModeStorage;
import org.firstinspires.ftc.teamcode.util.Pose2D;

import java.util.concurrent.atomic.AtomicBoolean;


@TeleOp (name="Main TeleOp", group="Competition")
@Config
public class MainTeleOp extends LinearOpMode {
    private RobotContainer robot; // main housing for robot subsystems

    private final boolean[] isTankDrive = {false};
    private final boolean[] launcherOnToggle = {false};
    private final boolean[] clawOpen = {false};
    private final boolean[] gripperOpen = {false};
    private final boolean[] isGlobalDrive = {true};
    private final int[]     intakeMode = {0};
    private final int[] wobbleGoalArmVeloc = {0};

    private final AtomicBoolean driveEnabled = new AtomicBoolean(true);

    private final double[] speedMultiplier = {1};

    public static double adjustAngle = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Initializing");

        // if the save state exists from auto then load it, otherwise init robot as normal
        if(InterOpModeStorage.savedRobot == null) {
            robot = new RobotContainer(this); // this initializes all subsystems in the robot
            //robot.localizer = new SwerveUnitAndIMULocalizer(robot.driveBase, robot.imu);
            robot.localizer = new DeadWheelsAndIMULocalizer(robot.xDeadWheel, robot.yDeadWheel, robot.imu);
            //robot.localizer.setStartPose(new Pose2D(24 * DistanceUnit.mPerInch, -2.5*24 * DistanceUnit.mPerInch, Math.PI/2));
            robot.localizer.setStartPose(new Pose2D(56.5 * DistanceUnit.mPerInch, -62 * DistanceUnit.mPerInch, Math.PI/2));

            robot.wobbleLifter.setClawClosed();
        } else {
            telemetry.addLine("Loading From Saved Robot...");
            telemetry.update();
            robot = InterOpModeStorage.savedRobot;
            robot.fixMotorsForTeleOp();
        }
        robot.localizer.setOpModeRunningListener(new Button() {
            @Override
            public boolean get() {
                return opModeIsActive();
            }
        });

        //robot.intake.setRingBlockerUp();

        telemetry.addLine("Robot Initialized:");
        telemetry.update();

        waitForStart();

        robot.intake.setRingBlockerDown();

        robot.localizer.startLocalizer();

        attachButtons();

        while(opModeIsActive()) {
            CommandScheduler.getInstance().run();

            Pose2D robotPose = robot.localizer.getRobotPose();
            double heading = robotPose.getTheta();

            double launchDist = Math.hypot(FieldConstants.LAUNCH_GOAL_CENTER.getY() - robotPose.getY(), FieldConstants.LAUNCH_GOAL_CENTER.getX() - robotPose.getX())/DistanceUnit.mPerInch;
            if(launcherOnToggle[0])
                //robot.launcher.turnOn(batteryVoltage);
                robot.launcher.setVelocityFromDistance(launchDist, gamepad2.back ? Launcher.LauncherTarget.POWERSHOT : Launcher.LauncherTarget.HIGH_GOAL);
                //robot.launcher.setLauncherVelocity(3700);
            else
                robot.launcher.setLauncherPower(0);
            telemetry.addData("launcherDistFromTarget", launchDist);

            robot.wobbleLifter.setLifterPosition(robot.wobbleLifter.getLifterPosition() + wobbleGoalArmVeloc[0]);

            if(driveEnabled.get()) {
                if (!isTankDrive[0]) {
                    if (isGlobalDrive[0]) {
                        Velocity desiredVelocity = new Velocity();
                        desiredVelocity.xVeloc = gamepad1.left_stick_y * DriveBaseConstants.MAX_DRIVE_VELOCITY * speedMultiplier[0];
                        desiredVelocity.yVeloc = gamepad1.left_stick_x * DriveBaseConstants.MAX_DRIVE_VELOCITY * speedMultiplier[0];
                        AngularVelocity desiredAngularVelocity = new AngularVelocity();
                        desiredAngularVelocity.zRotationRate = -gamepad1.right_stick_x * (float) Math.PI; // turning speed in rad/sec
                        robot.driveBase.setVelocityGlobal(desiredVelocity, desiredAngularVelocity, heading);
                    } else {
                        Velocity desiredVelocity = new Velocity();
                        desiredVelocity.xVeloc = gamepad1.left_stick_y * DriveBaseConstants.MAX_DRIVE_VELOCITY * speedMultiplier[0];
                        desiredVelocity.yVeloc = gamepad1.left_stick_x * DriveBaseConstants.MAX_DRIVE_VELOCITY * speedMultiplier[0];
                        AngularVelocity desiredAngularVelocity = new AngularVelocity();
                        desiredAngularVelocity.zRotationRate = -gamepad1.right_stick_x * (float) Math.PI; // turning speed in rad/sec
                        robot.driveBase.setVelocityLocal(desiredVelocity, desiredAngularVelocity);
                    }
                } else {
                    robot.driveBase.leftUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);
                    robot.driveBase.rightUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);
                    robot.driveBase.leftUnit.setDriveVelocity(-gamepad1.left_stick_y * DriveBaseConstants.MAX_DRIVE_VELOCITY * speedMultiplier[0]);
                    robot.driveBase.rightUnit.setDriveVelocity(-gamepad1.right_stick_y * DriveBaseConstants.MAX_DRIVE_VELOCITY * speedMultiplier[0]);
                }
            }

            if(clawOpen[0]) {
                robot.wobbleLifter.setClawOpen();
            } else {
                robot.wobbleLifter.setClawClosed();
            }

            if(intakeMode[0] == 1) {
                robot.intake.setIntakePower(1);
            } else if(intakeMode[0] == -1) {
                robot.intake.setIntakePower(-1);
            } else {
                robot.intake.setIntakePower(0);
            }

            Pose2D robotVeloc = robot.localizer.getRobotVelocity();

            telemetry.addData("launcherUpToSpeed", robot.launcher.isUpToSpeed());
            //telemetry.addData("launcherVelocity", robot.launcher.getLauncherVelocity());
            telemetry.addData("chassisVelocity", Math.hypot(robotVeloc.getX(), robotVeloc.getY()));
            //telemetry.addData("robotPose", robotPose.toString());
            telemetry.update();

            robot.driveStation.sendStringUDP("x~" + robotPose.getX() + ",y~" + robotPose.getY()  + ",rot~" + robotPose.getTheta());
        }
        robot.shutdown();
        InterOpModeStorage.savedRobot = null;
    }

    private void attachButtons() {
        Button clearCommandsButton = new Button() {
            @Override
            public boolean get() {
                return gamepad1.y;
            }
        };

        // button to toggle between tank drive mode, and holonomic swerve
        Button tankDriveToggleButton = new Button() {
            @Override
            public boolean get() {
                return gamepad1.a;
            }
        };

        // button to toggle between global swerve mode, and local swerve
        Button globalDriveToggleButton = new Button() {
            @Override
            public boolean get() {
                return gamepad1.b;
            }
        };

        // button to turn the launcher on and off
        Button launcherToggleButton = new Button() {
            @Override
            public boolean get() {
                return gamepad2.y;
            }
        };

        Button shiftSpeedUpButton = new Button() {
            @Override
            public boolean get() {
                return gamepad1.right_bumper;
            }
        };
        Button shiftSpeedDownButton = new Button() {
            @Override
            public boolean get() {
                return gamepad1.left_bumper;
            }
        };

        // buttons to choose how many rings to launch
        Button launch3RingsButton = new Button() {
            @Override
            public boolean get() {
                return gamepad2.x;
            }
        };
        Button launch2RingsButton = new Button() {
            @Override
            public boolean get() {
                return gamepad2.a;
            }
        };
        Button launch1Ring1Button = new Button() {
            @Override
            public boolean get() {
                return gamepad2.b;
            }
        };

        Button launchPowershots = new Button() {
            @Override
            public boolean get() {
                return gamepad2.back && gamepad2.right_bumper;
            }
        };

        Button targetHighGoal = new Button() {
            @Override
            public boolean get() {
                return gamepad2.right_trigger > .5;
            }
        };

        // buttons to operate wobble goal lifter
        Button wobbleGoalArmUpButton = new Button() {
            @Override
            public boolean get() {
                return gamepad2.dpad_up;
            }
        };
        Button wobbleGoalArmDownButton = new Button() {
            @Override
            public boolean get() {
                return gamepad2.dpad_down;
            }
        };
        Button wobbleGoalClawButton = new Button() {
            @Override
            public boolean get() {
                return gamepad2.left_bumper;
            }
        };

        // buttons to operate intake and transfer subsystem
        Button intakeInButton = new Button() {
            @Override
            public boolean get() {
                return gamepad1.right_trigger > .5;
            }
        };
        Button intakeOutButton = new Button() {
            @Override
            public boolean get() {
                return gamepad1.left_trigger > .5;
            }
        };

        // button to recalibrate the robot to the back left corner
        Button recalibrateBottomLeftButton = new Button() {
            @Override
            public boolean get() {
                return gamepad1.dpad_down;
            }
        };
        Button recalibrateTopLeftButton = new Button() {
            @Override
            public boolean get() {
                return gamepad1.dpad_up;
            }
        };


        clearCommandsButton.onButtonPressed(new RunnableCommand(() -> {
            CommandScheduler.getInstance().cancelAll();
        }));

        // now link each of the buttons to a command
        launcherToggleButton.onButtonPressed(new RunnableCommand(() -> {
            launcherOnToggle[0] = !launcherOnToggle[0];
        }));

        tankDriveToggleButton.onButtonPressed(new RunnableCommand(() -> {
            isTankDrive[0] = !isTankDrive[0];
        }));

        globalDriveToggleButton.onButtonPressed(new RunnableCommand(() -> {
            isGlobalDrive[0] = !isGlobalDrive[0];
        }));

        shiftSpeedUpButton.onButtonPressed(new RunnableCommand(() -> {
            if(!(speedMultiplier[0] >= 1)) {
                speedMultiplier[0] += 1.0 / 3;
            }
        }));

        shiftSpeedDownButton.onButtonPressed(new RunnableCommand(() -> {
            if(!(speedMultiplier[0] <= 1.0/3)) {
                speedMultiplier[0] -= 1.0 / 3;
            }
        }));

        launch3RingsButton.onButtonPressed(new LaunchRings(robot.launcher, 3));
        launch2RingsButton.onButtonPressed(new LaunchRings(robot.launcher, 2));
        launch1Ring1Button.onButtonPressed(new LaunchRings(robot.launcher, 1));

        launchPowershots.onButtonPressed(new RunnableCommand(() -> {
            driveEnabled.set(false);
        })
                .andThen(new TargetPose(robot.driveBase, robot.localizer, FieldConstants.PEG3_CENTER, 3000)
                .andThen(new LaunchRings(robot.launcher, 1))
                .andThen(new TargetPose(robot.driveBase, robot.localizer, FieldConstants.PEG2_CENTER, 3000)
                        .andThen(new LaunchRings(robot.launcher, 1))
                        .andThen(new TargetPose(robot.driveBase, robot.localizer, FieldConstants.PEG1_CENTER, 3000)
                                .andThen(new LaunchRings(robot.launcher, 1)
                                .andThen(new RunnableCommand(() -> {
                                    driveEnabled.set(true);
                                })))))));

        targetHighGoal.onButtonPressed(new RunnableCommand(() -> {
            driveEnabled.set(false);
        })
        .andThen(new TargetPose(robot.driveBase, robot.localizer, FieldConstants.LAUNCH_GOAL_CENTER, -adjustAngle*Math.PI/180, 2000)
                .andThen(new RunnableCommand(() -> {
                    driveEnabled.set(true);
                }))));

        //final int[] wobbleGoalArmState = {0};
        wobbleGoalArmUpButton.onButtonPressed(new RunnableCommand(() -> {
            /*if(wobbleGoalArmState[0] == 0) {
                robot.wobbleLifter.setLifterUp();
                wobbleGoalArmState[0] = 1;
            } else if(wobbleGoalArmState[0] == 1) {
                robot.wobbleLifter.setLifterDown();
                wobbleGoalArmState[0] = 2;
            } else if(wobbleGoalArmState[0] == 2) {
                robot.wobbleLifter.setLifterUp();
                wobbleGoalArmState[0] = 1;
            }*/
            wobbleGoalArmVeloc[0] = 130;
        }));
        wobbleGoalArmUpButton.onButtonReleased(new RunnableCommand(() -> {
            /*if(wobbleGoalArmState[0] == 0) {
                robot.wobbleLifter.setLifterUp();
                wobbleGoalArmState[0] = 1;
            } else if(wobbleGoalArmState[0] == 1) {
                robot.wobbleLifter.setLifterDown();
                wobbleGoalArmState[0] = 2;
            } else if(wobbleGoalArmState[0] == 2) {
                robot.wobbleLifter.setLifterUp();
                wobbleGoalArmState[0] = 1;
            }*/
            wobbleGoalArmVeloc[0] = 0;
        }));
        wobbleGoalArmDownButton.onButtonPressed(new RunnableCommand(() -> {
            /*if(wobbleGoalArmState[0] == 0) {
                robot.wobbleLifter.setLifterUp();
                wobbleGoalArmState[0] = 1;
            } else if(wobbleGoalArmState[0] == 1) {
                robot.wobbleLifter.setLifterDown();
                wobbleGoalArmState[0] = 2;
            } else if(wobbleGoalArmState[0] == 2) {
                robot.wobbleLifter.setLifterUp();
                wobbleGoalArmState[0] = 1;
            }*/
            wobbleGoalArmVeloc[0] = -130;
        }));
        wobbleGoalArmDownButton.onButtonReleased(new RunnableCommand(() -> {
            /*if(wobbleGoalArmState[0] == 0) {
                robot.wobbleLifter.setLifterUp();
                wobbleGoalArmState[0] = 1;
            } else if(wobbleGoalArmState[0] == 1) {
                robot.wobbleLifter.setLifterDown();
                wobbleGoalArmState[0] = 2;
            } else if(wobbleGoalArmState[0] == 2) {
                robot.wobbleLifter.setLifterUp();
                wobbleGoalArmState[0] = 1;
            }*/
            wobbleGoalArmVeloc[0] = 0;
        }));

        wobbleGoalClawButton.onButtonPressed(new RunnableCommand(() -> clawOpen[0] = !clawOpen[0]));

        intakeInButton.onButtonPressed(new RunnableCommand(() -> {
            intakeMode[0] = intakeMode[0] != 1 ? 1 : 0;
        }));

        intakeOutButton.onButtonPressed(new RunnableCommand(() -> {
            intakeMode[0] = intakeMode[0] != -1 ? -1 : 0;
        }));

        recalibrateBottomLeftButton.onButtonPressed(new RunnableCommand(() -> robot.localizer.setStartPose(new Pose2D(-1.2*12*DistanceUnit.mPerInch, -5.2*12*DistanceUnit.mPerInch, Math.PI/2))));

        recalibrateTopLeftButton.onButtonPressed(new RunnableCommand(() -> robot.localizer.setStartPose(new Pose2D(-1.2*12*DistanceUnit.mPerInch, 5.2*12*DistanceUnit.mPerInch, Math.PI/2))));

        CommandScheduler.getInstance().addButtons(clearCommandsButton,
                launcherToggleButton, tankDriveToggleButton, globalDriveToggleButton,
                shiftSpeedDownButton, shiftSpeedUpButton,
                launch3RingsButton, launch2RingsButton, launch1Ring1Button, launchPowershots,
                targetHighGoal,
                wobbleGoalArmUpButton, wobbleGoalArmDownButton, wobbleGoalClawButton,
                intakeInButton, intakeOutButton,
                recalibrateBottomLeftButton, recalibrateTopLeftButton);
    }

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }

    private double getBatteryVoltage() {
        double batteryVoltage = 0;

        for(VoltageSensor v : hardwareMap.voltageSensor) {
            double voltage = v.getVoltage();
            if(voltage > batteryVoltage) {
                batteryVoltage = voltage;
            }
        }
        return batteryVoltage;
    }
}
