package frc.lib.team2930;

import com.fasterxml.jackson.databind.JavaType;

import java.lang.annotation.Annotation;
import java.lang.reflect.*;
import java.util.*;

public class Reflection {
  private static final Object[]   c_emptyArgs     = new Object[0];
  private static final Class<?>[] c_emptyArgTypes = new Class<?>[0];

  public static class FieldAccessor {
    private final Field m_field;

    public FieldAccessor(Field f) {
      m_field = f;

      f.setAccessible(true);
    }

    // --//

    public String getName() {
      return m_field.getName();
    }

    public <T extends Annotation> T getAnnotation(Class<T> clz) {
      return m_field.getAnnotation(clz);
    }

    public Class<?> getDeclaringClass() {
      return m_field.getDeclaringClass();
    }

    public Field getNative() {
      return m_field;
    }

    public Class<?> getNativeType() {
      return m_field.getType();
    }

    public Type getTypeParameter(int i) {
      return Reflection.getTypeArgument(m_field.getGenericType(), i);
    }

    // --//

    public Object get(Object target) {
      try {
        return m_field.get(target);
      } catch (Exception e) {
        throw new RuntimeException(e);
      }
    }

    public void set(Object target, Object value) {
      try {
        m_field.set(target, value);
      } catch (Exception e) {
        throw new RuntimeException(e);
      }
    }
  }

  // --//

  public static Map<String, Field> collectFields(Class<?> clz) {
    var res = new HashMap<String, Field>();

    for (Class<?> clz2 = clz; clz2 != null; clz2 = clz2.getSuperclass()) {
      for (Field f : clz2.getDeclaredFields()) {
        res.put(f.getName(), f);
      }
    }

    return res;
  }

  public static <T> T newInstance(Class<T> clz) {
    try {
      Constructor<T> constr = clz.getConstructor(c_emptyArgTypes);
      return constr.newInstance(c_emptyArgs);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  public static Type getTypeArgument(Type type, int i) {
    Type typeArg = getTypeArgumentOrNull(type, i);
    if (typeArg == null) {
      throw Exceptions.newRuntimeException("Can't get type argument %d for type '%s'", i, type);
    }

    return typeArg;
  }

  public static <T> Class<T> getRawType(Type context, Type type) {
    if (type == null) {
      return null;
    }

    type = resolveGenericType(context, type);

    if (type instanceof Class<?>) {
      @SuppressWarnings("unchecked")
      Class<T> result = (Class<T>) type;
      return result;
    }

    if (type instanceof ParameterizedType) {
      ParameterizedType parameterizedType = (ParameterizedType) type;

      @SuppressWarnings("unchecked")
      Class<T> result = (Class<T>) parameterizedType.getRawType();
      return result;
    }

    if (type instanceof JavaType jsonType) {
      @SuppressWarnings("unchecked")
      Class<T> result = (Class<T>) jsonType.getRawClass();
      return result;
    }

    if (context != null) {
      throw Exceptions.newRuntimeException("Can't determine real type of %s [in context: %s]", type, context);
    } else {
      throw Exceptions.newRuntimeException("Can't determine real type of %s", type);
    }
  }

  public static <T> Class<T> getRawType(Type type) {
    return getRawType(null, type);
  }

  public static Type getTypeArgumentOrNull(Type type, int i) {
    if (type instanceof ParameterizedType paramType) {
      Type[] args = paramType.getActualTypeArguments();

      if (i >= 0 && i < args.length) {
        return args[i];
      }
    }

    if (type instanceof JavaType jsonType) {
      return jsonType.getBindings().getBoundType(i);
    }

    return null;
  }

  public static Type resolveGenericType(Type context, Type type) {
    if (type == null) {
      return null;
    }

    if (type instanceof Class<?>) {
      return type;
    }

    if (type instanceof ParameterizedType) {
      return type;
    }

    if (type instanceof TypeVariable<?>) {
      TypeVariable<?> type2 = (TypeVariable<?>) type;

      Object genDecl = type2.getGenericDeclaration();
      if (genDecl instanceof Class<?>) {
        Class<?> genDecl2 = (Class<?>) genDecl;

        TypeVariable<?>[] typeParams = genDecl2.getTypeParameters();
        for (int i = 0; i < typeParams.length; i++) {
          if (typeParams[i].equals(type2)) {
            if (context instanceof ParameterizedType) {
              ParameterizedType parameterizedType = (ParameterizedType) context;

              Type[] actualTypeArgs = parameterizedType.getActualTypeArguments();
              if (i < actualTypeArgs.length) {
                return actualTypeArgs[i];
              }
            }

            if (context instanceof JavaType) {
              JavaType jsonType = (JavaType) context;

              JavaType jsonTypeParam = jsonType.getBindings().getBoundType(i);
              if (jsonTypeParam != null) {
                return jsonTypeParam;
              }
            }
          }
        }
      }

      if (genDecl instanceof Method) {
        Method genDecl2 = (Method) genDecl;

        TypeVariable<?>[] typeParams = genDecl2.getTypeParameters();
        for (int i = 0; i < typeParams.length; i++) {
          if (typeParams[i].equals(type2)) {
            if (context instanceof ParameterizedType) {
              ParameterizedType parameterizedType = (ParameterizedType) context;

              Type[] actualTypeArgs = parameterizedType.getActualTypeArguments();
              if (i < actualTypeArgs.length) {
                return actualTypeArgs[i];
              }
            }

            if (context instanceof JavaType) {
              JavaType jsonType = (JavaType) context;

              JavaType jsonTypeParam = jsonType.getBindings().getBoundType(i);
              if (jsonTypeParam != null) {
                return jsonTypeParam;
              }
            }
          }
        }
      }

      Type[] bounds = type2.getBounds();
      if (bounds == null) {
        return Object.class;
      }

      if (bounds.length == 1) {
        return bounds[0];
      }
    }

    if (type instanceof JavaType) {
      return type;
    }

    if (context != null) {
      throw Exceptions.newRuntimeException("Can't determine real type of %s [in context: %s]", type, context);
    } else {
      throw Exceptions.newRuntimeException("Can't determine real type of %s", type);
    }
  }

}
