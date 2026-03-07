package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.Consumer;
import java.util.function.Function;

public class TalonFXWrapper {
  @FunctionalInterface
  public interface Allocator<T> {
    T create();
  }

  @FunctionalInterface
  public interface Getter<T> {
    StatusCode get(T value);
  }

  @FunctionalInterface
  public interface Setter<T> {
    StatusCode set(T value);
  }

  public static class State<T> {
    public T                   value;
    public StatusCode          status;
    public boolean             fetched;
    private final Allocator<T> creator;
    private final Getter<T>    getter;
    private final Setter<T>    setter;

    State(Allocator<T> creator, Getter<T> getter, Setter<T> setter) {
      this.creator = creator;
      this.getter  = getter;
      this.setter  = setter;
    }

    public void change(Consumer<T> consumer) {
      var v = get();
      consumer.accept(v);
      apply();
    }

    public void changeIfNeeded(Function<T, Boolean> consumer) {
      var v       = get();
      var refresh = consumer.apply(v);
      if (refresh) {
        apply();
      }
    }

    private T get() {
      if (!fetched) {
        value   = creator.create();
        status  = getter.get(value);
        fetched = status == StatusCode.OK;
      }

      return value;
    }

    private void apply() {
      if (value != null) {
        status = setter.set(value);
      }
    }
  }

  public final TalonFX io;

  public final State<TalonFXConfiguration>       talonFXConfiguration;
  public final State<MotorOutputConfigs>         motorOutputConfigs;
  public final State<CurrentLimitsConfigs>       currentLimitsConfigs;
  public final State<Slot0Configs>               slot0Configs;
  public final State<Slot1Configs>               slot1Configs;
  public final State<Slot2Configs>               slot2Configs;
  public final State<FeedbackConfigs>            feedbackConfigs;
  public final State<ClosedLoopRampsConfigs>     closedLoopRampsConfigs;
  public final State<OpenLoopRampsConfigs>       openLoopRampsConfigs;
  public final State<SoftwareLimitSwitchConfigs> softwareLimitSwitchConfigs;
  public final State<HardwareLimitSwitchConfigs> hardwareLimitSwitchConfigs;
  public final State<AudioConfigs>               audioConfigs;
  public final State<MotionMagicConfigs>         motionMagicConfigs;

  public TalonFXWrapper(TalonFX motor) {
    this.io = motor;

    this.talonFXConfiguration = new State<>(TalonFXConfiguration::new, io.getConfigurator()::refresh,
        io.getConfigurator()::apply);

    this.motorOutputConfigs = new State<>(MotorOutputConfigs::new, io.getConfigurator()::refresh,
        io.getConfigurator()::apply);

    this.currentLimitsConfigs = new State<>(CurrentLimitsConfigs::new, io.getConfigurator()::refresh,
        io.getConfigurator()::apply);

    this.slot0Configs = new State<>(Slot0Configs::new, io.getConfigurator()::refresh, io.getConfigurator()::apply);

    this.slot1Configs = new State<>(Slot1Configs::new, io.getConfigurator()::refresh, io.getConfigurator()::apply);

    this.slot2Configs = new State<>(Slot2Configs::new, io.getConfigurator()::refresh, io.getConfigurator()::apply);

    this.feedbackConfigs = new State<>(FeedbackConfigs::new, io.getConfigurator()::refresh,
        io.getConfigurator()::apply);

    this.closedLoopRampsConfigs = new State<>(ClosedLoopRampsConfigs::new, io.getConfigurator()::refresh,
        io.getConfigurator()::apply);

    this.openLoopRampsConfigs = new State<>(OpenLoopRampsConfigs::new, io.getConfigurator()::refresh,
        io.getConfigurator()::apply);

    this.softwareLimitSwitchConfigs = new State<>(SoftwareLimitSwitchConfigs::new, io.getConfigurator()::refresh,
        io.getConfigurator()::apply);

    this.hardwareLimitSwitchConfigs = new State<>(HardwareLimitSwitchConfigs::new, io.getConfigurator()::refresh,
        io.getConfigurator()::apply);

    this.audioConfigs = new State<>(AudioConfigs::new, io.getConfigurator()::refresh, io.getConfigurator()::apply);

    this.motionMagicConfigs = new State<>(MotionMagicConfigs::new, io.getConfigurator()::refresh,
        io.getConfigurator()::apply);
  }

  // --//

  public void setNeutralMode(NeutralModeValue value) {
    motorOutputConfigs.changeIfNeeded((config) -> {
      if (config.NeutralMode == value) {
        return false;
      }

      config.NeutralMode = value;
      return true;
    });
  }
}
