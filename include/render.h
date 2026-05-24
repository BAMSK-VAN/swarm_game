#ifndef RENDER_H
#define RENDER_H

#include <SDL2/SDL.h>
#include "config.h"
#include "types.h"
#include "globals.h"

void draw_point_centered(SDL_Renderer *renderer, int x, int y);
void k_draw_ellipse(SDL_Renderer *renderer, double x[NUM]);
void all_draw_ellipse(SDL_Renderer *renderer, double all_cx, double all_cy, double all_a, double all_b, double all_angle);
void draw_ellipse(SDL_Renderer *renderer);
void z_draw_ellipse(SDL_Renderer *renderer, double z_cx, double z_cy, double z_a, double z_b, double z_angle);
void draw_protect(SDL_Renderer *renderer);
void drawBirds(SDL_Renderer *renderer);
void draw_fov(SDL_Renderer *renderer, CameraDrone cd, double length);
void drawTP(SDL_Renderer *renderer);

#endif // RENDER_H