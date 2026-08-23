#pragma once

#if defined _WIN32 || defined __CYGWIN__
#ifdef MFR3DUO_MUJOCO_BUILDING_LIBRARY
#define MFR3DUO_MUJOCO_PUBLIC __declspec(dllexport)
#else
#define MFR3DUO_MUJOCO_PUBLIC __declspec(dllimport)
#endif
#else
#define MFR3DUO_MUJOCO_PUBLIC __attribute__((visibility("default")))
#endif
