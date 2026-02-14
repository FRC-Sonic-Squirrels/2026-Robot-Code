// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.ClimberConstants;
import frc.robot.RobotStates;
import frc.robot.RobotStates.MechState;
import frc.robot.subsystems.climber.Climber;
import java.util.function.BooleanSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
  private final String ROOT_TABLE = "Climb";

  private final Climber         climber;
  private final BooleanSupplier confirmButton;
  private final RobotStates     states;

  private boolean prevConfirm = true;
  private int     stage       = 0;

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Bool    confirm;
    public LoggerEntry.Integer stage;
  }

  public static class Tunables extends AutoRegisterTunables {
    @TunableEntry(defaultValue = 3.5)
    public LoggedTunableNumber Stage1AngleRot;

    @TunableEntry(defaultValue = 2.5)
    public LoggedTunableNumber Stage2AngleRot;

    @TunableEntry(defaultValue = 5.4)
    public LoggedTunableNumber Stage3AngleRot;

    @TunableEntry(defaultValue = 1.0)
    public LoggedTunableNumber Stage4AngleRot;

  }

  public final Logs      logs     = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);
  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE);

  private final Timer servoTimer = new Timer();

  /** Creates a new Climb. */
  public Climb(Climber climber, BooleanSupplier confirmButton, RobotStates states) {
    this.climber       = climber;
    this.confirmButton = confirmButton;
    this.states        = states;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    states.mechState      = MechState.ClimbPosition;
    states.mechanismInUse = true;
    states.intakeInUse    = true;
    climber.setServoAngle(ClimberConstants.SERVO_UNLOCK_ANGLE);
    stage       = 0;
    prevConfirm = true;
    servoTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean confirm = confirmButton.getAsBoolean();

    if (confirm && !prevConfirm) {
      stage++;
    }

    if (stage == 0) {
      Rotation2d target = Rotation2d.fromRotations(tunables.Stage1AngleRot.get());
      climber.setWinchAngle(target);
      if (climber.isWinchAtTargetAngle(target))
        stage++;
    } else if (stage == 1) {
      climber.setWinchAngle(Rotation2d.fromRotations(tunables.Stage2AngleRot.get()));
    } else if (stage == 2) {
      climber.setWinchAngle(Rotation2d.fromRotations(tunables.Stage3AngleRot.get()));
    } else {
      climber.setWinchAngle(Rotation2d.fromRotations(tunables.Stage4AngleRot.get()));
      servoTimer.start();
      if (servoTimer.get() > 1)
        climber.setServoAngle(ClimberConstants.SERVO_LOCK_ANGLE);
    }

    logs.stage.info(stage);
    logs.confirm.info(confirm);
    prevConfirm = confirm;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    states.mechanismInUse = false;
    states.intakeInUse    = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stage == 4;
  }
}
