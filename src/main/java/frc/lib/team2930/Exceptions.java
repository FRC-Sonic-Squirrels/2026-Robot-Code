/*
 * Copyright (C) 2017-, Optio3, Inc. All Rights Reserved.
 *
 * Proprietary & Confidential Information.
 */
package frc.lib.team2930;

public class Exceptions {
  public static RuntimeException newRuntimeException(String fmt, Object... args) {
    return new RuntimeException(String.format(fmt, args));
  }

  public static RuntimeException newIllegalArgumentException(String fmt, Object... args) {
    return new IllegalArgumentException(String.format(fmt, args));
  }
}
