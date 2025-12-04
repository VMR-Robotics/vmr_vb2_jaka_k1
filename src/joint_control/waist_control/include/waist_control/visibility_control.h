#ifndef WAIST_CONTROL__VISIBILITY_CONTROL_H_
#define WAIST_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define WAIST_CONTROL_EXPORT __attribute__((dllexport))
#define WAIST_CONTROL_IMPORT __attribute__((dllimport))
#else
#define WAIST_CONTROL_EXPORT __declspec(dllexport)
#define WAIST_CONTROL_IMPORT __declspec(dllimport)
#endif
#ifdef WAIST_CONTROL_BUILDING_DLL
#define WAIST_CONTROL_PUBLIC WAIST_CONTROL_EXPORT
#else
#define WAIST_CONTROL_PUBLIC WAIST_CONTROL_IMPORT
#endif
#define WAIST_CONTROL_PUBLIC_TYPE WAIST_CONTROL_PUBLIC
#define WAIST_CONTROL_LOCAL
#else
#define WAIST_CONTROL_EXPORT __attribute__((visibility("default")))
#define WAIST_CONTROL_IMPORT
#if __GNUC__ >= 4
#define WAIST_CONTROL_PUBLIC __attribute__((visibility("default")))
#define WAIST_CONTROL_LOCAL __attribute__((visibility("hidden")))
#else
#define WAIST_CONTROL_PUBLIC
#define WAIST_CONTROL_LOCAL
#endif
#define WAIST_CONTROL_PUBLIC_TYPE
#endif

#endif  // WAIST_CONTROL__VISIBILITY_CONTROL_H_
