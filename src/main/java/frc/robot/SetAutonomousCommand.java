/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;

public class SetAutonomousCommand {
    Constants constants;

    public SetAutonomousCommand(){
        constants = new Constants();
    }

    public Command getAutonomousCommand() {
        TrajectoryConfig config = new TrajectoryConfig(constants.maxVelocityMetersPerSecond,
        constants.maxAccelerationMetersPerSecondSqr);
        config.setKinematics(Robot.driveTrain.getKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(new Pose2d(),
        new Pose2d(1.5, 0, new Rotation2d())), config);

        RamseteCommand follow = new RamseteCommand(trajectory, Robot.driveTrain::getPose,
        new RamseteController(constants.kRamseteTuningB, constants.kRamseteTuningZeta),
        Robot.driveTrain.getFeedForward(), Robot.driveTrain.getKinematics(), Robot.driveTrain::getSpeeds,
        Robot.driveTrain.getLeftPidController(), Robot.driveTrain.getRightPidController(),
        Robot.driveTrain::setSpeed, Robot.driveTrain);

        return follow;
    }
}
