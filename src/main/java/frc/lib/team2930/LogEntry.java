package frc.lib.team2930;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

@Retention(RetentionPolicy.RUNTIME)
public @interface LogEntry {
  String name() default "";

  int updatePerSecond() default 50;
}
