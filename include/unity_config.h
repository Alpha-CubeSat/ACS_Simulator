#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

// Unity configuration macros
#define UNITY_DOUBLE_PRECISION 1e-12
#define UNITY_FLOAT_PRECISION 0.00001f
#define UNITY_OUTPUT_CHAR(a)
#define UNITY_OUTPUT_START()
#define UNITY_OUTPUT_FLUSH()
#define UNITY_OUTPUT_COMPLETE()

// Memory configurations
#define UNITY_POINTER_WIDTH 32
#define UNITY_EXCLUDE_STDDEF_H

// Support for 64-bit operations
#define UNITY_INCLUDE_64

#endif /* UNITY_CONFIG_H */