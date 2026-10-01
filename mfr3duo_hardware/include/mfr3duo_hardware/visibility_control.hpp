#ifndef MFR3DUO_HARDWARE__VISIBILITY_CONTROL_HPP_
#define MFR3DUO_HARDWARE__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MFR3DUO_HARDWARE_EXPORT __attribute__((dllexport))
#define MFR3DUO_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define MFR3DUO_HARDWARE_EXPORT __declspec(dllexport)
#define MFR3DUO_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef MFR3DUO_HARDWARE_BUILDING_LIBRARY
#define MFR3DUO_HARDWARE_PUBLIC MFR3DUO_HARDWARE_EXPORT
#else
#define MFR3DUO_HARDWARE_PUBLIC MFR3DUO_HARDWARE_IMPORT
#endif
#define MFR3DUO_HARDWARE_PUBLIC_TYPE MFR3DUO_HARDWARE_PUBLIC
#define MFR3DUO_HARDWARE_LOCAL
#else
#define MFR3DUO_HARDWARE_EXPORT __attribute__((visibility("default")))
#define MFR3DUO_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define MFR3DUO_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define MFR3DUO_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define MFR3DUO_HARDWARE_PUBLIC
#define MFR3DUO_HARDWARE_LOCAL
#endif
#define MFR3DUO_HARDWARE_PUBLIC_TYPE
#endif

#endif  // MFR3DUO_HARDWARE__VISIBILITY_CONTROL_HPP_
