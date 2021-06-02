package org.firstinspires.ftc.teamcode.GameOpModes.CommandBuilders;

import com.qualcomm.robotcore.exception.RobotProtocolException;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.commands.LaunchRings;
import org.firstinspires.ftc.teamcode.commands.RunnableCommand;
import org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.TargetPose;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;
import org.firstinspires.ftc.teamcode.trackers.PurePursuitTracker;
import org.firstinspires.ftc.teamcode.util.EndPose2D;
import org.firstinspires.ftc.teamcode.util.PointPath;
import org.firstinspires.ftc.teamcode.util.Pose2D;
import org.firstinspires.ftc.teamcode.util.Trajectory;

/**
 * The AutonomousBuilder class generates SequentialCommandGroups
 * containing the code that should be run for various autonomous
 * actions.
 */
public class AutonomousBuilder {

    public static SequentialCommandGroup buildZeroRingAutoLeftLine(RobotContainer robot) {
        Pose2D startPoint = robot.localizer.getRobotPose().copy();

        Trajectory path1 = new PointPath();
        Trajectory path2_part1 = new PointPath();
        Trajectory path2_part2 = new PointPath();
        Trajectory path3 = new PointPath();

        PurePursuitTracker tracker1 = new PurePursuitTracker(path1);
        tracker1.setLookAheadDistance(.1, .1);
        tracker1.setMaxVelocity(.8);

        path1.addPoint(startPoint);
        path1.addPoint(new Pose2D(0.38735, -0.61595, Math.PI/2));
        path1.addPoint(new Pose2D(0.6731, 0.10795, Math.PI/2));
        path1.addPoint(new EndPose2D(1.2827, 0.32385, Math.PI/2));

        path2_part1.addPoint(new Pose2D(1.127, 0.32385, Math.PI/2));
        path2_part1.addPoint(new Pose2D(1.26365, -0.62229997, 0));
        path2_part2.addPoint(new Pose2D(1.26365, -0.62229997, 0));
        path2_part2.addPoint(new EndPose2D(1.2089, -0.85, 0));

        path3.addPoint(new Pose2D(1.3589, -0.9, 0));
        path3.addPoint(new Pose2D(1.26365, -0.62229997, 0));
        path3.addPoint(new EndPose2D(1.2827, 0.32385, Math.PI/2));

        return new RunnableCommand(() -> {
            Pose2D robotPose = robot.localizer.getRobotPose();
            double launchDist = Math.hypot(FieldConstants.LAUNCH_GOAL_CENTER.getY() - robotPose.getY(), FieldConstants.LAUNCH_GOAL_CENTER.getX() - robotPose.getX())/ DistanceUnit.mPerInch;
            robot.launcher.setVelocityFromDistance(launchDist+2, Launcher.LauncherTarget.HIGH_GOAL);
            //robot.launcher.setLauncherVelocity(3410);

            sleep(750);
        })
         .andThen(new LaunchRings(robot.launcher, 3, 400)
                 .andThen(new RunnableCommand(() -> {
                     robot.launcher.setLauncherPower(0);
                 }))
         .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
         .andThen(new RunnableCommand(() -> {
             robot.driveBase.setVelocityToZero();
         }))
         .andThen(new RunnableCommand(() -> {
             robot.wobbleLifter.setLifterDown();
             sleep(800);
             robot.wobbleLifter.setClawOpen();
             sleep(500);
             robot.wobbleLifter.setLifterUp();
             sleep(500);
             robot.wobbleLifter.setClawClosed();

             tracker1.setTrajectory(path2_part1);
             tracker1.setMaxVelocity(.35);
         })
         .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
         .andThen(new RunnableCommand(() -> {
             robot.wobbleLifter.setLifterDown();
             robot.wobbleLifter.setClawOpen();

             tracker1.setTrajectory(path2_part2);
             tracker1.setMaxVelocity(.35);
         })
         .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
         .andThen(new RunnableCommand(() -> {
             robot.driveBase.setVelocityToZero();
             robot.wobbleLifter.setClawClosed();
             sleep(800);
             robot.wobbleLifter.setLifterUp();

             tracker1.setTrajectory(path3);
             tracker1.setMaxVelocity(.35);
         })
         .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
         .andThen(new RunnableCommand(() -> {
             robot.driveBase.setVelocityToZero();
             robot.wobbleLifter.setLifterDown();
             try {
                 Thread.sleep(800);
             } catch (InterruptedException e) {
                 e.printStackTrace();
             }
             robot.wobbleLifter.setClawOpen();
             try {
                 Thread.sleep(500);
             } catch (InterruptedException e) {
                 e.printStackTrace();
             }
             robot.wobbleLifter.setLifterUp();
             try {
                 Thread.sleep(500);
             } catch (InterruptedException e) {
                 e.printStackTrace();
             }
             robot.wobbleLifter.setClawClosed();
         }))))))))));
    }

    public static SequentialCommandGroup buildOneRingAuto(RobotContainer robot) {
        Pose2D startPoint = robot.localizer.getRobotPose().copy();

        Trajectory path1 = new PointPath();

        PurePursuitTracker tracker = new PurePursuitTracker(path1);
        tracker.setLookAheadDistance(.1, .1);
        tracker.setMaxVelocity(.35);

        path1.addPoint(startPoint);
        path1.addPoint(new Pose2D(0.38735, -0.61595, Math.PI/2));
        path1.addPoint(new Pose2D(0.635, 0.9271, Math.PI/2));
        return new RunnableCommand(() -> {
            Pose2D robotPose = robot.localizer.getRobotPose();
            double launchDist = Math.hypot(FieldConstants.LAUNCH_GOAL_CENTER.getY() - robotPose.getY(), FieldConstants.LAUNCH_GOAL_CENTER.getX() - robotPose.getX())/ DistanceUnit.mPerInch;
            robot.launcher.setVelocityFromDistance(launchDist+2, Launcher.LauncherTarget.HIGH_GOAL);
            //robot.launcher.setLauncherVelocity(3410);
            try {
                Thread.sleep(750);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        })
                .andThen(new LaunchRings(robot.launcher, 3, 400)
                        .andThen(new RunnableCommand(() -> {
                            robot.launcher.setLauncherPower(0);
                        }))
                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker)
                                .andThen(new RunnableCommand(() -> {
                                    robot.driveBase.setVelocityToZero();
                                }))
                                .andThen(new RunnableCommand(() -> {
                                    robot.wobbleLifter.setLifterDown();
                                    try {
                                        Thread.sleep(800);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    robot.wobbleLifter.setClawOpen();
                                    try {
                                        Thread.sleep(500);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    robot.wobbleLifter.setLifterUp();
                                    try {
                                        Thread.sleep(500);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    robot.wobbleLifter.setClawClosed();
                                }))));
    }

    public static SequentialCommandGroup buildFourRingAuto(RobotContainer robot) {
        Pose2D startPoint = robot.localizer.getRobotPose().copy();

        Trajectory path1 = new PointPath();

        PurePursuitTracker tracker = new PurePursuitTracker(path1);
        tracker.setLookAheadDistance(.1, .1);
        tracker.setMaxVelocity(.35);

        path1.addPoint(startPoint);
        path1.addPoint(new Pose2D(0.38735, -0.61595, Math.PI/2));
        path1.addPoint(new Pose2D(0.635, 0.9271, Math.PI/2));
        path1.addPoint(new Pose2D(1.2445999, 1.56845, Math.PI/2));

        return new RunnableCommand(() -> {
            Pose2D robotPose = robot.localizer.getRobotPose();
            double launchDist = Math.hypot(FieldConstants.LAUNCH_GOAL_CENTER.getY() - robotPose.getY(), FieldConstants.LAUNCH_GOAL_CENTER.getX() - robotPose.getX())/ DistanceUnit.mPerInch;
            robot.launcher.setVelocityFromDistance(launchDist, Launcher.LauncherTarget.HIGH_GOAL);
            //robot.launcher.setLauncherVelocity(3410);

            try {
                Thread.sleep(750);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        })
                .andThen(new LaunchRings(robot.launcher, 3, 400)
                        .andThen(new RunnableCommand(() -> {
                            robot.launcher.setLauncherPower(0);
                        }))
                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker)
                                .andThen(new RunnableCommand(() -> {
                                    robot.driveBase.setVelocityToZero();
                                }))
                                .andThen(new RunnableCommand(() -> {
                                    robot.wobbleLifter.setLifterDown();
                                    try {
                                        Thread.sleep(800);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    robot.wobbleLifter.setClawOpen();
                                    try {
                                        Thread.sleep(500);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    robot.wobbleLifter.setLifterUp();
                                    try {
                                        Thread.sleep(500);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    robot.wobbleLifter.setClawClosed();
                                }))));
    }

    public static SequentialCommandGroup buildZeroRingAutoRightLine(RobotContainer robot) {
        Pose2D startPoint = robot.localizer.getRobotPose().copy();

        Trajectory path1 = new PointPath(new Pose2D[] {
                startPoint,
                new EndPose2D(1.2827, 0.29305, 1.91986)
        });

        Trajectory path2_part1 = new PointPath(new Pose2D[] {
                new Pose2D(1.26365, 0.3175, Math.PI/2),
                new Pose2D(1.27, 0.3175, Math.PI/2),
                new Pose2D(1.2573, 0.29845, 0)
        });

        Trajectory path2_part2 = new PointPath(new Pose2D[] {
                new Pose2D(0.9271, -0.4064, 0),
                new EndPose2D(0.60, -.9, 0)
        });

        Trajectory path3 = new PointPath(new Pose2D[] {
                new Pose2D(0.60, -1.02235, 0),
                new EndPose2D(1.25095, 0.3683, 1.6799989)
        });

        Trajectory path4 = new PointPath(new Pose2D[] {
                new Pose2D(1.25095, 0.3683, 1.6799989),
                new EndPose2D(0, 0.3683, 1.6799989)
        });

        PurePursuitTracker tracker1 = new PurePursuitTracker(path1);
        tracker1.setLookAheadDistance(.1, .1);
        tracker1.setMaxVelocity(1.2);

        return new RunnableCommand(() -> {
            Pose2D robotPose = robot.localizer.getRobotPose();
            double launchDist = Math.hypot(FieldConstants.LAUNCH_GOAL_CENTER.getY() - robotPose.getY(), FieldConstants.LAUNCH_GOAL_CENTER.getX() - robotPose.getX())/ DistanceUnit.mPerInch;
            robot.launcher.setVelocityFromDistance(launchDist+5.5, Launcher.LauncherTarget.HIGH_GOAL);
            //robot.launcher.setLauncherVelocity(3400);
            //robot.launcher.setVelocityFromDistance(DistanceUnit.mPerInch * 136, Launcher.LauncherTarget.HIGH_GOAL);

            try {
                Thread.sleep(1250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        })
                .andThen(new LaunchRings(robot.launcher, 3, 400)
                        .andThen(new RunnableCommand(() -> {
                            robot.launcher.setLauncherPower(0);
                        }))
                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
                                .andThen(new RunnableCommand(() -> {
                                    robot.driveBase.setVelocityToZero();
                                })
                                .andThen(new RunnableCommand(() -> {
                                    robot.wobbleLifter.setLifterDown();
                                    robot.intake.setRingBlockerDown();
                                    sleep(800);
                                    robot.wobbleLifter.setClawOpen();
                                    sleep(500);
                                    robot.wobbleLifter.setLifterUp();

                                    tracker1.setTrajectory(path2_part1);
                                    tracker1.setMaxVelocity(1.2);
                                })
                                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
                                                .andThen(new RunnableCommand(() -> {
                                                    robot.wobbleLifter.setLifterDown();
                                                    robot.wobbleLifter.setClawOpen();

                                                    tracker1.setTrajectory(path2_part2);
                                                    tracker1.setMaxVelocity(1.2);
                                                })
                                                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
                                                                .andThen(new RunnableCommand(() -> {
                                                                    robot.driveBase.setVelocityToZero();
                                                                    robot.wobbleLifter.setClawClosed();
                                                                    try {
                                                                        Thread.sleep(1200);
                                                                    } catch (InterruptedException e) {
                                                                        e.printStackTrace();
                                                                    }
                                                                    robot.wobbleLifter.setLifterUp();

                                                                    tracker1.setTrajectory(path3);
                                                                    tracker1.setMaxVelocity(1.2);
                                                                })
                                                                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
                                                                                .andThen(new RunnableCommand(() -> {
                                                                                    robot.driveBase.setVelocityToZero();
                                                                                    robot.wobbleLifter.setLifterDown();
                                                                                    try {
                                                                                        Thread.sleep(800);
                                                                                    } catch (InterruptedException e) {
                                                                                        e.printStackTrace();
                                                                                    }
                                                                                    robot.wobbleLifter.setClawOpen();
                                                                                    try {
                                                                                        Thread.sleep(500);
                                                                                    } catch (InterruptedException e) {
                                                                                        e.printStackTrace();
                                                                                    }
                                                                                    robot.wobbleLifter.setLifterUp();
                                                                                    try {
                                                                                        Thread.sleep(500);
                                                                                    } catch (InterruptedException e) {
                                                                                        e.printStackTrace();
                                                                                    }
                                                                                    robot.wobbleLifter.setClawClosed();

                                                                                    tracker1.setTrajectory(path4);
                                                                                    tracker1.setMaxVelocity(1.2);
                                                                                })
                                                                                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
                                                                                        .andThen(() -> {
                                                                                            robot.driveBase.setVelocityToZero();
                                                                                            robot.driveBase.rightUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);
                                                                                            robot.driveBase.leftUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);
                                                                                            sleep(2000);
                                                                                        }))))))))))));
    }

    public static SequentialCommandGroup buildOneRingAutoRightLine(RobotContainer robot) {
        Pose2D startPoint = robot.localizer.getRobotPose().copy();

        Trajectory path1 = new PointPath(new Pose2D[] {
                startPoint,
                new Pose2D(1.47955, -0.80645, Math.PI/2),
                new Pose2D(1.30175, -0.19685, Math.PI/2),
                new Pose2D(1.0795, 0.3429, 3.3599973),
                new Pose2D(1.0287, 0.77945, 3.3599973)
        });

        Trajectory path2 = new PointPath(new Pose2D[] {
                new Pose2D(1.0287, 0.67945, 3.3599973),
                new Pose2D(0.55245, 0.0254, 1),
                new Pose2D(0.42545, -0.33655, 0.20999998)

        });

        Trajectory path2_part2 = new PointPath(new Pose2D[] {
                new Pose2D(0.4318, -0.6096, 0.32999998),
                new EndPose2D(0.7098, -0.97789997, 0.20999998)
        });

        Trajectory path3 = new PointPath(new Pose2D[] {
                new Pose2D(0.50165, -0.84455, 0.26999998),
                new Pose2D(0.4572, -0.27305, 1.529999),
                new Pose2D(0.61595, 0.38099998, 2.4599981),
                new Pose2D(0.75025, 0.7207, 2.669998)
        });

        Trajectory path4 = new PointPath(new Pose2D[] {
                new Pose2D(0.73025, 0.5207, 2.669998),
                new EndPose2D(0.10795, 0.3302, Math.PI/2)
        });


        PurePursuitTracker tracker1 = new PurePursuitTracker(path1);
        tracker1.setLookAheadDistance(.1, .1);
        tracker1.setMaxVelocity(1.2);

        return new RunnableCommand(() -> {
            Pose2D robotPose = robot.localizer.getRobotPose();
            double launchDist = Math.hypot(FieldConstants.LAUNCH_GOAL_CENTER.getY() - robotPose.getY(), FieldConstants.LAUNCH_GOAL_CENTER.getX() - robotPose.getX())/ DistanceUnit.mPerInch;
            robot.launcher.setVelocityFromDistance(launchDist+2, Launcher.LauncherTarget.HIGH_GOAL);
            //robot.launcher.setLauncherVelocity(3425);

            try {
                Thread.sleep(1255);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        })
                .andThen(new LaunchRings(robot.launcher, 3, 400)
                        .andThen(new RunnableCommand(() -> {
                            robot.launcher.setLauncherPower(0);
                        }))
                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
                                .andThen(new RunnableCommand(() -> {
                                    robot.driveBase.setVelocityToZero();
                                }))
                                .andThen(new RunnableCommand(() -> {
                                    robot.wobbleLifter.setLifterDown();
                                    sleep(400);
                                    robot.wobbleLifter.setClawOpen();
                                    sleep(400);
                                    robot.wobbleLifter.setLifterUp();

                                    tracker1.setTrajectory(path2);
                                    tracker1.setMaxVelocity(1.2);
                                })
                                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
                                                .andThen(new RunnableCommand(() -> {
                                                    robot.intake.setRingBlockerDown();

                                                    robot.wobbleLifter.setLifterDown();
                                                    robot.wobbleLifter.setClawOpen();

                                                    tracker1.setTrajectory(path2_part2);
                                                    tracker1.setMaxVelocity(1.2);
                                                })
                                                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
                                                                .andThen(new RunnableCommand(() -> {
                                                                    robot.driveBase.setVelocityToZero();
                                                                    robot.wobbleLifter.setClawClosed();
                                                                    try {
                                                                        Thread.sleep(1200);
                                                                    } catch (InterruptedException e) {
                                                                        e.printStackTrace();
                                                                    }
                                                                    robot.wobbleLifter.setLifterUp();

                                                                    tracker1.setTrajectory(path3);
                                                                    tracker1.setMaxVelocity(1.2);
                                                                })
                                                                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
                                                                                .andThen(new RunnableCommand(() -> {
                                                                                    robot.driveBase.setVelocityToZero();
                                                                                    robot.wobbleLifter.setLifterDown();
                                                                                    try {
                                                                                        Thread.sleep(800);
                                                                                    } catch (InterruptedException e) {
                                                                                        e.printStackTrace();
                                                                                    }
                                                                                    robot.wobbleLifter.setClawOpen();
                                                                                    try {
                                                                                        Thread.sleep(500);
                                                                                    } catch (InterruptedException e) {
                                                                                        e.printStackTrace();
                                                                                    }
                                                                                    robot.wobbleLifter.setLifterUp();
                                                                                    try {
                                                                                        Thread.sleep(500);
                                                                                    } catch (InterruptedException e) {
                                                                                        e.printStackTrace();
                                                                                    }
                                                                                    robot.wobbleLifter.setClawClosed();

                                                                                    tracker1.setTrajectory(path4);
                                                                                    tracker1.setMaxVelocity(1.2);
                                                                                })
                                                                                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
                                                                                                .andThen(() -> {
                                                                                                    robot.driveBase.setVelocityToZero();
                                                                                                })))))))))));
    }

    public static SequentialCommandGroup buildFourRingAutoRightLine(RobotContainer robot) {
        Pose2D startPoint = robot.localizer.getRobotPose().copy();

        Trajectory path1 = new PointPath(new Pose2D[] {
                startPoint,
                new Pose2D(1.3716, 1.2827, 2.639998)
        });

        Trajectory path2 = new PointPath(new Pose2D[] {
                new Pose2D(1.3716, 1.2827, 2.639998),
                new Pose2D(0.59055, -0.15875, -2.6747584E-8)
        });

        Trajectory path2_part2 = new PointPath(new Pose2D[] {
                new Pose2D(0.59055, -0.15875, -2.6747584E-8),
                new EndPose2D(0.72085, -0.8817, 0)
        });

        Trajectory path3 = new PointPath(new Pose2D[] {
                new Pose2D(0.4826, -0.889, 0.453608885641),
                new Pose2D(1.3033, 1.397, Math.PI/2+Math.PI/4)
        });

        Trajectory path4 = new PointPath(new Pose2D[] {
                new Pose2D(1.0033, 1.397, Math.PI/2+Math.PI/4),
                new Pose2D(0.61595, .30, 1.68359838564)
        });


        PurePursuitTracker tracker1 = new PurePursuitTracker(path1);
        tracker1.setLookAheadDistance(.1, .1);
        tracker1.setMaxVelocity(1.2);

        return new RunnableCommand(() -> {
            Pose2D robotPose = robot.localizer.getRobotPose();
            double launchDist = Math.hypot(FieldConstants.LAUNCH_GOAL_CENTER.getY() - robotPose.getY(), FieldConstants.LAUNCH_GOAL_CENTER.getX() - robotPose.getX())/ DistanceUnit.mPerInch;
            robot.launcher.setVelocityFromDistance(launchDist+4, Launcher.LauncherTarget.HIGH_GOAL);
            //robot.launcher.setLauncherVelocity(3422);

            sleep(1255);
        })
                .andThen(new LaunchRings(robot.launcher, 3, 400)
                .andThen(new RunnableCommand(() -> {
                    robot.launcher.setLauncherVelocity(0);
                })

                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
                                .andThen(new RunnableCommand(() -> {
                                    robot.driveBase.setVelocityToZero();
                                }))
                                .andThen(new RunnableCommand(() -> {
                                    robot.wobbleLifter.setLifterDown();
                                    sleep(650);
                                    robot.wobbleLifter.setClawOpen();
                                    sleep(400);
                                    robot.wobbleLifter.setLifterUp();

                                    tracker1.setTrajectory(path2);
                                    tracker1.setMaxVelocity(1.2);
                                })
                                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
                                                .andThen(new RunnableCommand(() -> {
                                                    robot.intake.setRingBlockerDown();

                                                    robot.wobbleLifter.setLifterDown();
                                                    robot.wobbleLifter.setClawOpen();

                                                    tracker1.setTrajectory(path2_part2);
                                                    tracker1.setMaxVelocity(1.2);
                                                })
                                                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
                                                                .andThen(new RunnableCommand(() -> {
                                                                    robot.driveBase.setVelocityToZero();
                                                                    robot.wobbleLifter.setClawClosed();
                                                                    sleep(1200);
                                                                    robot.wobbleLifter.setLifterUp();

                                                                    tracker1.setTrajectory(path3);
                                                                    tracker1.setMaxVelocity(1.2);
                                                                })
                                                                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
                                                                                .andThen(new RunnableCommand(() -> {
                                                                                    robot.driveBase.setVelocityToZero();
                                                                                    robot.launcher.setLauncherVelocity(0);
                                                                                    robot.wobbleLifter.setLifterDown();
                                                                                    try {
                                                                                        Thread.sleep(500);
                                                                                    } catch (InterruptedException e) {
                                                                                        e.printStackTrace();
                                                                                    }
                                                                                    robot.wobbleLifter.setClawOpen();
                                                                                    try {
                                                                                        Thread.sleep(500);
                                                                                    } catch (InterruptedException e) {
                                                                                        e.printStackTrace();
                                                                                    }
                                                                                    robot.wobbleLifter.setLifterUp();
                                                                                    try {
                                                                                        Thread.sleep(100);
                                                                                    } catch (InterruptedException e) {
                                                                                        e.printStackTrace();
                                                                                    }
                                                                                    robot.wobbleLifter.setClawClosed();

                                                                                    tracker1.setTrajectory(path4);
                                                                                    tracker1.setMaxVelocity(1.2);
                                                                                })
                                                                                        .andThen(new FollowPath(robot.driveBase, robot.localizer, tracker1)
                                                                                                .andThen(() -> {
                                                                                                    robot.driveBase.setVelocityToZero();
                                                                                                }))))))))))));
    }

    /**
     * helper method
     */
    private static void sleep(long duration) {
        try {
            Thread.sleep(duration);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
