#ifndef YAOBU_CONTROL__VISIBILITY_CONTROL_H_
#define YAOBU_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define YAOBU_CONTROL_EXPORT __attribute__((dllexport))
#define YAOBU_CONTROL_IMPORT __attribute__((dllimport))
#else
#define YAOBU_CONTROL_EXPORT __declspec(dllexport)
#define YAOBU_CONTROL_IMPORT __declspec(dllimport)
#endif
#ifdef YAOBU_CONTROL_BUILDING_DLL
#define YAOBU_CONTROL_PUBLIC YAOBU_CONTROL_EXPORT
#else
#define YAOBU_CONTROL_PUBLIC YAOBU_CONTROL_IMPORT
#endif
#define YAOBU_CONTROL_PUBLIC_TYPE YAOBU_CONTROL_PUBLIC
#define YAOBU_CONTROL_LOCAL
#else
#define YAOBU_CONTROL_EXPORT __attribute__((visibility("default")))
#define YAOBU_CONTROL_IMPORT
#if __GNUC__ >= 4
#define YAOBU_CONTROL_PUBLIC __attribute__((visibility("default")))
#define YAOBU_CONTROL_LOCAL __attribute__((visibility("hidden")))
#else
#define YAOBU_CONTROL_PUBLIC
#define YAOBU_CONTROL_LOCAL
#endif
#define YAOBU_CONTROL_PUBLIC_TYPE
#endif

#endif  // YAOBU_CONTROL__VISIBILITY_CONTROL_H_
