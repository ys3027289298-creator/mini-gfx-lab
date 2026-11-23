#include "canvas.h"

#include <stdio.h>
#include <string.h>

void canvas_init(Canvas* c) {
    c->w = CANVAS_W;
    c->h = CANVAS_H;
    canvas_clear(c);
}

void canvas_clear(Canvas* c) {
    for (int y = 0; y < CANVAS_H; y++) {
        memset(c->px[y], ' ', CANVAS_W);
        c->px[y][CANVAS_W] = '\0';
    }
}

void canvas_fill(Canvas* c, char ch) {
    for (int y = 0; y < CANVAS_H; y++) {
        memset(c->px[y], ch, CANVAS_W);
        c->px[y][CANVAS_W] = '\0';
    }
}

void canvas_put(Canvas* c, int x, int y, char ch) {
    if (x < 0 || y < 0 || x >= c->w || y >= c->h) return;
    c->px[y][x] = ch;
}

void canvas_text(Canvas* c, int x, int y, const char* s) {
    if (!s) return;
    for (int i = 0; s[i] && x + i < c->w; i++) canvas_put(c, x + i, y, s[i]);
}

void canvas_clear_band(Canvas* c, int y0, int rows) {
    if (y0 < 0) { rows += y0; y0 = 0; }
    if (y0 + rows > c->h) rows = c->h - y0;
    for (int y = y0; y < y0 + rows; y++) {
        memset(c->px[y], ' ', CANVAS_W);
        c->px[y][CANVAS_W] = '\0';
    }
}

void canvas_blit_band(Canvas* dst, const Canvas* src, int y0, int rows) {
    if (y0 < 0) { rows += y0; y0 = 0; }
    if (rows > src->h) rows = src->h;
    if (y0 + rows > dst->h) rows = dst->h - y0;
    for (int y = 0; y < rows; y++)
        memcpy(dst->px[y0 + y], src->px[y], (size_t)src->w);
}

void canvas_dump(const Canvas* c, void* file) {
    FILE* f = (FILE*)file;
    if (!f) f = stdout;
    for (int x = 0; x < c->w + 2; x++) fputc('-', f);
    fputc('\n', f);
    for (int y = 0; y < c->h; y++) {
        fputc('|', f);
        fwrite(c->px[y], 1, (size_t)c->w, f);
        fputc('|', f);
        fputc('\n', f);
    }
    for (int x = 0; x < c->w + 2; x++) fputc('-', f);
    fputc('\n', f);
}
