package frc.lib.team2930;

import frc.lib.team6328.LoggedTunableNumber;

public class TunableNumberGroup {
  public static <T extends AutoRegisterTunables> T newAutoTunable(Class<T> clz, String path) {
    var instance = Reflection.newInstance(clz);

    var fields = Reflection.collectFields(clz);
    fields.forEach((f, v) -> {
      var accessor = new Reflection.FieldAccessor(v);

      double defaultValue = Double.NaN;
      var    logName      = f.substring(0, 1).toUpperCase() + f.substring(1);
      var    anno         = v.getAnnotation(TunableEntry.class);
      if (anno != null) {
        defaultValue = anno.defaultValue();
        if (!anno.name().isEmpty()) {
          logName = anno.name();
        }
      }

      String subPath = path + "/" + logName;

      var t = accessor.getNativeType();
      if (AutoRegisterTunables.class.isAssignableFrom(t)) {
        // noinspection unchecked
        accessor.set(instance, newAutoTunable((Class<T>) t, subPath));
      } else if (t == LoggedTunableNumber.class) {
        if (Double.isNaN(defaultValue)) {
          accessor.set(instance, new LoggedTunableNumber(subPath));
        } else {
          accessor.set(instance, new LoggedTunableNumber(subPath, defaultValue));
        }
      }
    });

    return instance;
  }
}
