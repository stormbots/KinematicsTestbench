// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Lerp;
import frc.robot.commands.MatrixMath;
import frc.robot.commands.setArm;
import frc.robot.subsystems.ArmBrake;
import frc.robot.subsystems.ArmstrongArmKinematics;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ArmstrongArmKinematics.RetractSolenoidPosition;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ArmstrongArmKinematics arm = new ArmstrongArmKinematics();
  public ArmBrake armBrake = new ArmBrake();

  private final CommandJoystick driver = new CommandJoystick(1);
  private final CommandJoystick operator = new CommandJoystick(0);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
  //    new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    arm.armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    arm.armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // arm.setDefaultCommand(
    //   new setArm(90, 0, 0, 0, arm)
    //   );
    // Configure the trigger bindings
    configureBindings();
    MatrixMath matrix = new MatrixMath(4, 4);
    System.out.println("Hello world!");
    System.out.println(matrix);
    SmartDashboard.putString("matrix", matrix.toString());

    armBrake.setDefaultCommand(new RunCommand(
      ()->{
        arm.setRetractBrake(RetractSolenoidPosition.DISENGAGED);
      }, armBrake)
      .finallyDo((bool)->arm.setRetractBrake(RetractSolenoidPosition.ENGAGED))
      );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // driver.button(2).onTrue(new InstantCommand(()->{
    //   ik=arm.inverseKinematics(0, 35+22, 90);
    //   SmartDashboard.putNumber("ik/extDistance", ik[0]);
    //   SmartDashboard.putNumber("ik/extAngle", ik[1]);
    // }));

    operator.button(1).whileTrue(
    new RunCommand(()->{
      var joy = operator.getRawAxis(3);
      var inches = Lerp.lerp(joy, -1, 1, 12, 36);
      var ik=arm.inverseKinematics(34, inches, 0);
      SmartDashboard.putNumber("ik/extDistance", ik[0]);
      SmartDashboard.putNumber("ik/extAngle", ik[1]);
      SmartDashboard.putNumber("ik/extInches", inches);
      SmartDashboard.putNumber("ik/setArmWristAngle", ik[2]);
      Supplier<double[]> var = ()->ik;
    
      }
    ));

    // operator.button(2).InstantCommand(
    //   new setArm(
    //   ()->arm.setArm(90, 0, 0, 0, arm)
    // );

    

    operator.button(1).whileTrue(
      new setArm(
      ()->arm.inverseKinematics(
          34, 
          Lerp.lerp(operator.getRawAxis(3), -1,  1, 12, 36), 
          0
        ), 
        ()->0, 
        arm)
    );
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
