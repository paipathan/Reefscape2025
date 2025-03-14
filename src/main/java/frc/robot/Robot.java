// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ButtonBoard.Action;
import frc.robot.subsystems.Vision.Camera;

public class Robot extends TimedRobot {
        XboxController controller;
        ButtonBoard board;

        Container container;

        StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().
                getStructTopic("Robot Pose", Pose2d.struct).publish();
                

        public Robot() {
                controller = new XboxController(Constants.controllerID);
                board = new ButtonBoard(Constants.boardID);

                container = new Container();
                CommandScheduler.getInstance().cancelAll();
        }

        @Override
        public void robotPeriodic() {
                CommandScheduler.getInstance().run();

                container.getDrivetrain().updateRobotHeight(container.getElevator().getPosition());
                container.updateRobotPose(container.getVision().getEstimate(Camera.Front));

                container.updateLEDs();

                publisher.set(container.getRobotPose());
        }

        @Override
        public void autonomousInit() {}

        @Override
        public void autonomousPeriodic() {}

        @Override
        public void autonomousExit() {}

        @Override
        public void teleopInit() {}

        @Override
        public void teleopPeriodic() {
                if (controller.getXButtonPressed()) container.getDrivetrain().seedFieldCentric();

                if (board.getButtonPressed(Action.Mode_Coral)) container.modeCoral();
                if (board.getButtonPressed(Action.Mode_Algae)) container.modeAlgae();

                if (controller.getAButtonPressed()) container.stow().schedule();
                

                if (board.getButtonPressed(Action.Target_Low)) container.targetLow();
                if (board.getButtonPressed(Action.Target_Medium)) container.targetMedium();
                if (board.getButtonPressed(Action.Target_High)) container.targetHigh();

                if (board.getButtonPressed(Action.Align_Left)) container.driveToPose(Constants.Vision.leftPoses[0]).schedule();
                if (board.getButtonPressed(Action.Align_Right)) container.driveToPose(Constants.Vision.rightPoses[0]).schedule();
                if (board.getButtonPressed(Action.Align_Center)) container.driveToPose(Constants.Vision.centerPoses[0]).schedule();

                container.driveJoysticks(controller.getLeftX(), controller.getLeftY(), controller.getRightX()).schedule();

                if (controller.getLeftBumperButtonPressed()) container.runIntake().schedule();
                if (controller.getRightBumperButtonPressed()) container.runOuttake().schedule();
        }

        @Override
        public void teleopExit() {}
}
