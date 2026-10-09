// Minimal stand-in for the three SDL calls the upstream scene library makes (asset lookup and
// logging), so upstream/scenes.cpp compiles unmodified without SDL.
#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>

// Directory that contains assets/, with a trailing slash. Set by sdf_set_resource_directory.
extern char sdfResourceDirectory[4096];

inline char *SDL_GetBasePath() { return strdup(sdfResourceDirectory); }
inline void SDL_free(void *memory) { free(memory); }
#define SDL_Log(...) (fprintf(stderr, __VA_ARGS__), fputc('\n', stderr))
