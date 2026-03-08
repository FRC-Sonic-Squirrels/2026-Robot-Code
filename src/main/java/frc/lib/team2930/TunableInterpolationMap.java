package frc.lib.team2930;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.lib.team6328.LoggedTunableNumber;

import java.util.HashMap;
import java.util.Map;

public class TunableInterpolationMap {
  static class Entry {
    LoggedTunableNumber tunable;
    double              value;
  }

  private final String                     path;
  private final InterpolatingDoubleTreeMap interpolator = new InterpolatingDoubleTreeMap();
  private final Map<Double, Entry>         entries      = new HashMap<>();

  public TunableInterpolationMap(String path, Map<Double, Double> value) {
    this.path = path;

    value.forEach((k, v) -> {
      this.interpolator.put(k, v);
      var entry = new Entry();
      entry.tunable = new LoggedTunableNumber(String.format("InterpolationMaps/%s/%f", path, k), v);
      entry.value   = v;
      entries.put(k, entry);
    });
  }

  public double get(double v) {
    for (var pair : this.entries.entrySet()) {
      var entry = pair.getValue();
      var vNew  = entry.tunable.get();
      var vOld  = entry.value;
      if (vNew != vOld) {
        rebuildInterpolator();
        break;
      }
    }

    return interpolator.get(v);
  }

  private void rebuildInterpolator() {
    interpolator.clear();

    this.entries.forEach((k, v) -> {
      v.value = v.tunable.get();
      this.interpolator.put(k, v.value);
      System.out.printf("InterpolationMap %s: %f -> %f\n", path, k, v.value);
    });
  }
}
