package frc.lib.team2930;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

@Retention(RetentionPolicy.RUNTIME)
public @interface TunableEntry {
  String name() default "";

  double defaultValue() default 0;
}
