#ifndef HEAD_CONTROL__VISIBILITY_CONTROL_H_
#define HEAD_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define HEAD_CONTROL_EXPORT __attribute__((dllexport))
#define HEAD_CONTROL_IMPORT __attribute__((dllimport))
#else
#define HEAD_CONTROL_EXPORT __declspec(dllexport)
#define HEAD_CONTROL_IMPORT __declspec(dllimport)
#endif
#ifdef HEAD_CONTROL_BUILDING_DLL
#define HEAD_CONTROL_PUBLIC HEAD_CONTROL_EXPORT
#else
#define HEAD_CONTROL_PUBLIC HEAD_CONTROL_IMPORT
#endif
#define HEAD_CONTROL_PUBLIC_TYPE HEAD_CONTROL_PUBLIC
#define HEAD_CONTROL_LOCAL
#else
#define HEAD_CONTROL_EXPORT __attribute__((visibility("default")))
#define HEAD_CONTROL_IMPORT
#if __GNUC__ >= 4
#define HEAD_CONTROL_PUBLIC __attribute__((visibility("default")))
#define HEAD_CONTROL_LOCAL __attribute__((visibility("hidden")))
#else
#define HEAD_CONTROL_PUBLIC
#define HEAD_CONTROL_LOCAL
#endif
#define HEAD_CONTROL_PUBLIC_TYPE
#endif

#endif  // HEAD_CONTROL__VISIBILITY_CONTROL_H_
