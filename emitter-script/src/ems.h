/* ems: a tiny runtime emitter description language.
   Parse once at load time, then run the same pool/quota/priority rules as
   the hand-written particle system. */
#ifndef EMS_H
#define EMS_H

#include <stddef.h>

#define EMS_MAX_EMITTERS 16
#define EMS_MAX_CURVES 16
#define EMS_MAX_TICKS 1024
#define EMS_MAX_POOL 65536
#define EMS_MAX_NAME 32
#define EMS_MAX_QUOTA 1000000

typedef enum {
    EMS_SHAPE_LINEAR = 0,
    EMS_SHAPE_QUAD_IN,
    EMS_SHAPE_QUAD_OUT,
    EMS_SHAPE_CUBIC_IO,
    EMS_SHAPE_STEP
} EmsShape;

typedef struct {
    char name[EMS_MAX_NAME];
    EmsShape shape;
    float from;
    float to;
} EmsCurve;

typedef enum {
    EMS_VALUE_CONST = 0,
    EMS_VALUE_CURVE
} EmsValueKind;

typedef struct {
    EmsValueKind kind;
    float number;                  /* EMS_VALUE_CONST */
    char ref[EMS_MAX_NAME];        /* EMS_VALUE_CURVE, as written */
    int curve;                     /* resolved curve index once compiled */
    int line, col;                 /* location of the value, for diagnostics */
} EmsValue;

typedef struct {
    char name[EMS_MAX_NAME];
    int priority;
    EmsValue quota;
    EmsValue pos[2];
    EmsValue vel[2];
    EmsValue life;
} EmsEmitter;

typedef struct {
    int pool;                      /* capacity of the runtime pool */
    int budget;                    /* particles per tick across all emitters */
    int ticks;
    float dt;
    EmsCurve curves[EMS_MAX_CURVES];
    int curve_count;
    EmsEmitter emitters[EMS_MAX_EMITTERS];
    int emitter_count;
} EmsProgram;

typedef struct {
    int line, col;                 /* 1-based; col counts bytes */
    char message[160];
} EmsError;

typedef struct {
    int frame_alive[EMS_MAX_TICKS]; /* alive count after each tick */
    int frames;                    /* ticks actually run */
    int high_water;                /* peak alive right after spawning */
    int spawned[EMS_MAX_EMITTERS];
    int expired[EMS_MAX_EMITTERS];
    int alive[EMS_MAX_EMITTERS];   /* alive after the last tick */
    double sum_x, sum_y, sum_age;  /* final-alive digest, additive */
} EmsResult;

/* Compile src. Returns 0 on success, -1 on the first lexical, syntax or
   semantic error. err->line/err->col then point at the offending token. */
int ems_compile(const char* src, EmsProgram* out, EmsError* err);

/* Execute a compiled program. Returns 0 on success, -1 on invalid input. */
int ems_run(const EmsProgram* prog, EmsResult* out);

/* Render "file:line:col: error: ..." plus the source line and a caret.
   Returns the number of characters written (excluding the terminator). */
int ems_format_error(const char* src, const char* filename,
                     const EmsError* err, char* out, size_t cap);

const char* ems_shape_name(EmsShape shape);

#endif
