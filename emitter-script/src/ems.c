/* ems: parser, resolver and executor for the emitter description language.

   Grammar (line oriented; '#' starts a comment that runs to end of line):

       program   := stmt*
       stmt      := 'pool'  INT
                  | 'budget' INT
                  | 'ticks'  INT
                  | 'dt'     NUMBER
                  | 'curve'  NAME '(' NUMBER ',' NUMBER ',' SHAPE ')'
                  | 'emitter' NAME 'priority' INT '{' field* '}'
       field     := 'quota' VALUE
                  | 'pos'   VALUE ',' VALUE
                  | 'vel'   VALUE ',' VALUE
                  | 'life'  VALUE
       VALUE     := NUMBER | '@' NAME
       SHAPE     := 'linear' | 'quad_in' | 'quad_out' | 'cubic_io' | 'step'
       NUMBER    := [+-]? (DIGIT+ ['.' DIGIT*] | '.' DIGIT+) ([eE] [+-]? DIGIT*)?

   Each emitter must give quota, pos, vel and life exactly once.

   Values are evaluated at emission time from t = tick * dt (tick starts at 0).
   A reference @name evaluates a curve; non-step curves interpolate from `from`
   to `to` over a one-unit period, so a reference to an emitter's own start time
   sweeps the curve over the first second. step returns `from` until t >= 1 and
   `to` afterwards. Fractional quotas are carried across ticks, so the quota is
   the exact mean spawn rate per tick.

   Per tick, emitters spawn in descending priority order (script order breaks
   ties), each spending min(quota(t), remaining frame budget). Spawned particles
   have life = life(t) and age by dt on the same tick; expired particles free
   their pool slots before the next tick. This mirrors particle/src/particle.c.

   The compiler is allocation-free apart from a small scratch array and uses no
   recursion, so parse cost is linear in script size. The run time allocates the
   pool on the heap; the script's pool setting is only a capacity. */
#include "ems.h"

#include <ctype.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---------- tokens ---------- */

typedef enum {
    TK_EOF = 0, TK_NAME, TK_NUMBER, TK_REF,
    TK_POOL, TK_BUDGET, TK_TICKS, TK_DT, TK_CURVE, TK_EMITTER,
    TK_QUOTA, TK_POS, TK_VEL, TK_LIFE, TK_PRIORITY,
    TK_LINEAR, TK_QUAD_IN, TK_QUAD_OUT, TK_CUBIC_IO, TK_STEP,
    TK_LBRACE, TK_RBRACE, TK_LPAREN, TK_RPAREN, TK_COMMA
} TokKind;

typedef struct {
    TokKind kind;
    const char* text;
    int len;
    int line, col;   /* 1-based; col counts bytes */
    double num;
} Token;

typedef struct {
    const char* src;
    const char* p;
    int line, col;
    Token tok;
    Token last;      /* most recently consumed token */
} Lexer;

static const struct { const char* word; TokKind kind; } KEYWORDS[] = {
    { "pool", TK_POOL },         { "budget", TK_BUDGET },
    { "ticks", TK_TICKS },       { "dt", TK_DT },
    { "curve", TK_CURVE },       { "emitter", TK_EMITTER },
    { "quota", TK_QUOTA },       { "pos", TK_POS },
    { "vel", TK_VEL },           { "life", TK_LIFE },
    { "priority", TK_PRIORITY }, { "linear", TK_LINEAR },
    { "quad_in", TK_QUAD_IN },   { "quad_out", TK_QUAD_OUT },
    { "cubic_io", TK_CUBIC_IO }, { "step", TK_STEP },
};

static void lex_advance(Lexer* lx) {
    if (*lx->p == '\n') { lx->line++; lx->col = 1; }
    else lx->col++;
    lx->p++;
}

static void lex_skip_space(Lexer* lx) {
    for (;;) {
        char c = *lx->p;
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') { lex_advance(lx); continue; }
        if (c == '#') { while (*lx->p && *lx->p != '\n') lex_advance(lx); continue; }
        break;
    }
}

/* returns 0, or -1 when the identifier is longer than any name can be */
static int lex_ident(Lexer* lx) {
    const char* start = lx->p;
    while (isalnum((unsigned char)*lx->p) || *lx->p == '_') lex_advance(lx);
    int len = (int)(lx->p - start);
    for (size_t i = 0; i < sizeof(KEYWORDS) / sizeof(KEYWORDS[0]); i++) {
        if ((int)strlen(KEYWORDS[i].word) == len &&
            memcmp(KEYWORDS[i].word, start, (size_t)len) == 0) {
            lx->tok.kind = KEYWORDS[i].kind;
            lx->tok.text = start;
            lx->tok.len = len;
            return 0;
        }
    }
    if (len > EMS_MAX_NAME - 1) return -1;
    lx->tok.kind = TK_NAME;
    lx->tok.text = start;
    lx->tok.len = len;
    return 0;
}

static int lex_number(Lexer* lx) {
    const char* start = lx->p;
    if (*lx->p == '+' || *lx->p == '-') lex_advance(lx);
    int digits = 0;
    while (isdigit((unsigned char)*lx->p)) { lex_advance(lx); digits++; }
    if (*lx->p == '.') {
        lex_advance(lx);
        while (isdigit((unsigned char)*lx->p)) { lex_advance(lx); digits++; }
    }
    if (digits == 0) return -1;
    if (*lx->p == 'e' || *lx->p == 'E') {
        const char* save = lx->p;
        int save_line = lx->line, save_col = lx->col;
        lex_advance(lx);
        if (*lx->p == '+' || *lx->p == '-') lex_advance(lx);
        int exp_digits = 0;
        while (isdigit((unsigned char)*lx->p)) { lex_advance(lx); exp_digits++; }
        if (exp_digits == 0) { lx->p = save; lx->line = save_line; lx->col = save_col; }
    }
    char buf[64];
    int len = (int)(lx->p - start);
    if (len >= (int)sizeof(buf)) return -1;
    memcpy(buf, start, (size_t)len);
    buf[len] = '\0';
    double num = strtod(buf, NULL);
    if (!isfinite(num)) return -1;
    lx->tok.kind = TK_NUMBER;
    lx->tok.text = start;
    lx->tok.len = len;
    lx->tok.num = num;
    return 0;
}

/* 0 on success; -1 on a malformed token, with lx->tok still holding its start */
static int lex_next(Lexer* lx) {
    lex_skip_space(lx);
    lx->tok.line = lx->line;
    lx->tok.col = lx->col;
    lx->tok.num = 0.0;
    char c = *lx->p;
    if (c == '\0') {
        lx->tok.kind = TK_EOF;
        lx->tok.text = lx->p;
        lx->tok.len = 0;
        return 0;
    }
    if (isalpha((unsigned char)c) || c == '_') return lex_ident(lx);
    if (isdigit((unsigned char)c) ||
        ((c == '+' || c == '-') && isdigit((unsigned char)lx->p[1])) ||
        (c == '.' && isdigit((unsigned char)lx->p[1])))
        return lex_number(lx);
    if (c == '@') {
        lex_advance(lx);
        const char* start = lx->p;
        while (isalnum((unsigned char)*lx->p) || *lx->p == '_') lex_advance(lx);
        int len = (int)(lx->p - start);
        if (len == 0 || len > EMS_MAX_NAME - 1) return -1;
        lx->tok.kind = TK_REF;
        lx->tok.text = start;
        lx->tok.len = len;
        return 0;
    }
    TokKind kind;
    switch (c) {
    case '{': kind = TK_LBRACE; break;
    case '}': kind = TK_RBRACE; break;
    case '(': kind = TK_LPAREN; break;
    case ')': kind = TK_RPAREN; break;
    case ',': kind = TK_COMMA; break;
    default: return -1;
    }
    lx->tok.kind = kind;
    lx->tok.text = lx->p;
    lx->tok.len = 1;
    lex_advance(lx);
    return 0;
}

/* ---------- parser ---------- */

typedef struct {
    Lexer lx;
    EmsProgram* out;
    char (*names)[EMS_MAX_NAME];   /* curve names, parallel to out->curves */
    int curve_line[EMS_MAX_CURVES];
    int emitter_line[EMS_MAX_EMITTERS];
    int failed;
    EmsError* err;
} Parser;

static void fail_at(Parser* ps, int line, int col, const char* fmt, ...) {
    if (ps->failed) return;
    ps->failed = 1;
    ps->err->line = line;
    ps->err->col = col;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(ps->err->message, sizeof(ps->err->message), fmt, ap);
    va_end(ap);
}

static void describe(const Token* t, char* out, size_t cap) {
    switch (t->kind) {
    case TK_EOF: snprintf(out, cap, "end of input"); break;
    case TK_NUMBER: snprintf(out, cap, "number '%.*s'", t->len, t->text); break;
    case TK_NAME: snprintf(out, cap, "'%.*s'", t->len, t->text); break;
    case TK_REF: snprintf(out, cap, "reference '@%.*s'", t->len, t->text); break;
    case TK_LBRACE: snprintf(out, cap, "'{'"); break;
    case TK_RBRACE: snprintf(out, cap, "'}'"); break;
    case TK_LPAREN: snprintf(out, cap, "'('"); break;
    case TK_RPAREN: snprintf(out, cap, "')'"); break;
    case TK_COMMA: snprintf(out, cap, "','"); break;
    default: snprintf(out, cap, "keyword '%.*s'", t->len, t->text); break;
    }
}

static void fail_expected(Parser* ps, const char* what) {
    char got[96];
    describe(&ps->lx.tok, got, sizeof(got));
    fail_at(ps, ps->lx.tok.line, ps->lx.tok.col, "expected %s, found %s", what, got);
}

/* fetch the next token; records the consumed one in lx->last */
static int step(Parser* ps) {
    if (ps->failed) return 0;
    ps->lx.last = ps->lx.tok;
    if (lex_next(&ps->lx) != 0) {
        fail_at(ps, ps->lx.tok.line, ps->lx.tok.col, "invalid token");
        return 0;
    }
    return 1;
}

static int expect(Parser* ps, TokKind kind, const char* what) {
    if (ps->failed) return 0;
    if (ps->lx.tok.kind != kind) { fail_expected(ps, what); return 0; }
    return 1;
}

static int take_name(Parser* ps, char out[EMS_MAX_NAME], const char* what) {
    if (!expect(ps, TK_NAME, what)) return 0;
    memcpy(out, ps->lx.tok.text, (size_t)ps->lx.tok.len);
    out[ps->lx.tok.len] = '\0';
    return step(ps);
}

static int take_int(Parser* ps, int lo, int hi, int* out) {
    if (!expect(ps, TK_NUMBER, "an integer")) return 0;
    const Token* t = &ps->lx.tok;
    if (t->num != floor(t->num) || t->num < (double)lo || t->num > (double)hi) {
        fail_at(ps, t->line, t->col, "expected an integer in [%d, %d], found '%.*s'",
                lo, hi, t->len, t->text);
        return 0;
    }
    *out = (int)t->num;
    return step(ps);
}

static int take_float(Parser* ps, float* out) {
    if (!expect(ps, TK_NUMBER, "a number")) return 0;
    *out = (float)ps->lx.tok.num;
    return step(ps);
}

/* a missing '}' must stop at the next statement instead of swallowing it */
static int starts_statement(TokKind kind) {
    switch (kind) {
    case TK_POOL: case TK_BUDGET: case TK_TICKS: case TK_DT:
    case TK_CURVE: case TK_EMITTER: case TK_EOF:
        return 1;
    default:
        return 0;
    }
}

static int parse_value(Parser* ps, EmsValue* v) {
    memset(v, 0, sizeof(*v));
    v->curve = -1;
    v->line = ps->lx.tok.line;
    v->col = ps->lx.tok.col;
    if (ps->lx.tok.kind == TK_NUMBER) {
        v->kind = EMS_VALUE_CONST;
        v->number = (float)ps->lx.tok.num;
        return step(ps);
    }
    if (ps->lx.tok.kind == TK_REF) {
        v->kind = EMS_VALUE_CURVE;
        memcpy(v->ref, ps->lx.tok.text, (size_t)ps->lx.tok.len);
        v->ref[ps->lx.tok.len] = '\0';
        return step(ps);
    }
    fail_expected(ps, "a number or a curve reference like @fade");
    return 0;
}

static int parse_shape(Parser* ps, EmsShape* out) {
    switch (ps->lx.tok.kind) {
    case TK_LINEAR: *out = EMS_SHAPE_LINEAR; break;
    case TK_QUAD_IN: *out = EMS_SHAPE_QUAD_IN; break;
    case TK_QUAD_OUT: *out = EMS_SHAPE_QUAD_OUT; break;
    case TK_CUBIC_IO: *out = EMS_SHAPE_CUBIC_IO; break;
    case TK_STEP: *out = EMS_SHAPE_STEP; break;
    default:
        fail_expected(ps, "a curve shape (linear, quad_in, quad_out, cubic_io, step)");
        return 0;
    }
    return step(ps);
}

static int parse_curve(Parser* ps) {
    EmsProgram* out = ps->out;
    char name[EMS_MAX_NAME];
    Token name_tok = ps->lx.tok;
    if (!take_name(ps, name, "a curve name")) return 0;
    for (int i = 0; i < out->curve_count; i++) {
        if (strcmp(ps->names[i], name) == 0) {
            fail_at(ps, name_tok.line, name_tok.col,
                    "curve '%s' already defined on line %d", name, ps->curve_line[i]);
            return 0;
        }
    }
    if (out->curve_count >= EMS_MAX_CURVES) {
        fail_at(ps, name_tok.line, name_tok.col, "too many curves (limit %d)", EMS_MAX_CURVES);
        return 0;
    }
    float from, to;
    EmsShape shape;
    if (!expect(ps, TK_LPAREN, "'(' after the curve name")) return 0;
    if (!step(ps)) return 0;
    if (!take_float(ps, &from)) return 0;
    if (!expect(ps, TK_COMMA, "',' between the curve endpoints")) return 0;
    if (!step(ps)) return 0;
    if (!take_float(ps, &to)) return 0;
    if (!expect(ps, TK_COMMA, "',' before the curve shape")) return 0;
    if (!step(ps)) return 0;
    if (!parse_shape(ps, &shape)) return 0;
    if (!expect(ps, TK_RPAREN, "')' to close the curve")) return 0;
    if (!step(ps)) return 0;

    EmsCurve* c = &out->curves[out->curve_count];
    memset(c, 0, sizeof(*c));
    memcpy(c->name, name, sizeof(name));
    c->shape = shape;
    c->from = from;
    c->to = to;
    memcpy(ps->names[out->curve_count], name, sizeof(name));
    ps->curve_line[out->curve_count] = name_tok.line;
    out->curve_count++;
    return 1;
}

static int parse_emitter_fields(Parser* ps, EmsEmitter* em) {
    int fields = 0;
    const int required = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
    for (;;) {
        TokKind kind = ps->lx.tok.kind;
        if (kind == TK_RBRACE) {
            if (!step(ps)) return 0;
            if (fields != required) {
                fail_at(ps, ps->lx.last.line, ps->lx.last.col,
                        "emitter '%s' needs quota, pos, vel and life", em->name);
                return 0;
            }
            return 1;
        }
        if (starts_statement(kind)) {
            fail_expected(ps, "'}' to close the emitter block");
            return 0;
        }
        Token field = ps->lx.tok;
        int bit;
        switch (field.kind) {
        case TK_QUOTA: bit = 1 << 0; break;
        case TK_POS:   bit = 1 << 1; break;
        case TK_VEL:   bit = 1 << 2; break;
        case TK_LIFE:  bit = 1 << 3; break;
        default:
            fail_at(ps, field.line, field.col, "unknown field '%.*s'", field.len, field.text);
            return 0;
        }
        if (!step(ps)) return 0;
        if (fields & bit) {
            fail_at(ps, field.line, field.col, "duplicate field '%.*s'", field.len, field.text);
            return 0;
        }
        fields |= bit;
        if (field.kind == TK_QUOTA) {
            if (!parse_value(ps, &em->quota)) return 0;
        } else if (field.kind == TK_LIFE) {
            if (!parse_value(ps, &em->life)) return 0;
        } else {
            EmsValue* pair = field.kind == TK_POS ? em->pos : em->vel;
            if (!parse_value(ps, &pair[0])) return 0;
            if (!expect(ps, TK_COMMA, "',' between the two values")) return 0;
            if (!step(ps)) return 0;
            if (!parse_value(ps, &pair[1])) return 0;
        }
    }
}

static int parse_emitter(Parser* ps) {
    EmsProgram* out = ps->out;
    Token name_tok = ps->lx.tok;
    EmsEmitter em;
    memset(&em, 0, sizeof(em));
    em.quota.curve = em.pos[0].curve = em.pos[1].curve =
        em.vel[0].curve = em.vel[1].curve = em.life.curve = -1;
    if (!take_name(ps, em.name, "an emitter name")) return 0;
    for (int i = 0; i < out->emitter_count; i++) {
        if (strcmp(out->emitters[i].name, em.name) == 0) {
            fail_at(ps, name_tok.line, name_tok.col,
                    "emitter '%s' already defined on line %d", em.name, ps->emitter_line[i]);
            return 0;
        }
    }
    if (out->emitter_count >= EMS_MAX_EMITTERS) {
        fail_at(ps, name_tok.line, name_tok.col, "too many emitters (limit %d)", EMS_MAX_EMITTERS);
        return 0;
    }
    if (!expect(ps, TK_PRIORITY, "'priority'")) return 0;
    if (!step(ps)) return 0;
    if (!take_int(ps, INT_MIN, INT_MAX, &em.priority)) return 0;
    if (!expect(ps, TK_LBRACE, "'{' to open the emitter fields")) return 0;
    if (!step(ps)) return 0;
    if (!parse_emitter_fields(ps, &em)) return 0;
    out->emitters[out->emitter_count] = em;
    ps->emitter_line[out->emitter_count] = name_tok.line;
    out->emitter_count++;
    return 1;
}

static int parse_statement(Parser* ps) {
    EmsProgram* out = ps->out;
    Token kw = ps->lx.tok;
    switch (kw.kind) {
    case TK_POOL:
        if (!step(ps)) return 0;
        return take_int(ps, 1, EMS_MAX_POOL, &out->pool);
    case TK_BUDGET:
        if (!step(ps)) return 0;
        return take_int(ps, 0, EMS_MAX_QUOTA, &out->budget);
    case TK_TICKS:
        if (!step(ps)) return 0;
        return take_int(ps, 1, EMS_MAX_TICKS, &out->ticks);
    case TK_DT:
        if (!step(ps)) return 0;
        if (!take_float(ps, &out->dt)) return 0;
        if (!(out->dt >= 0.0f)) {
            fail_at(ps, ps->lx.last.line, ps->lx.last.col, "dt must not be negative");
            return 0;
        }
        return 1;
    case TK_CURVE:
        if (!step(ps)) return 0;
        return parse_curve(ps);
    case TK_EMITTER:
        if (!step(ps)) return 0;
        return parse_emitter(ps);
    case TK_NAME:
        fail_at(ps, kw.line, kw.col,
                "unknown statement '%.*s'; expected pool, budget, ticks, dt, curve or emitter",
                kw.len, kw.text);
        return 0;
    case TK_LBRACE:
    case TK_RBRACE:
    case TK_LPAREN:
    case TK_RPAREN:
    case TK_COMMA:
    case TK_REF:
    case TK_QUOTA:
    case TK_POS:
    case TK_VEL:
    case TK_LIFE:
    case TK_PRIORITY:
    case TK_LINEAR:
    case TK_QUAD_IN:
    case TK_QUAD_OUT:
    case TK_CUBIC_IO:
    case TK_STEP:
        fail_at(ps, kw.line, kw.col,
                "unexpected %.*s; expected a statement (pool, budget, ticks, dt, curve, emitter)",
                kw.len, kw.text);
        return 0;
    default:
        fail_expected(ps, "a statement (pool, budget, ticks, dt, curve, emitter)");
        return 0;
    }
}

static int resolve_refs(Parser* ps) {
    EmsProgram* out = ps->out;
    for (int i = 0; i < out->emitter_count; i++) {
        EmsEmitter* em = &out->emitters[i];
        EmsValue* values[] = { &em->quota, &em->pos[0], &em->pos[1],
                               &em->vel[0], &em->vel[1], &em->life };
        for (size_t k = 0; k < sizeof(values) / sizeof(values[0]); k++) {
            EmsValue* v = values[k];
            if (v->kind != EMS_VALUE_CURVE) continue;
            int found = -1;
            for (int c = 0; c < out->curve_count; c++)
                if (strcmp(ps->names[c], v->ref) == 0) { found = c; break; }
            if (found < 0) {
                fail_at(ps, v->line, v->col, "unknown curve '@%s'", v->ref);
                return 0;
            }
            v->curve = found;
        }
    }
    return 1;
}

int ems_compile(const char* src, EmsProgram* out, EmsError* err) {
    if (!src || !out || !err) return -1;
    memset(out, 0, sizeof(*out));
    out->dt = 1.0f;
    out->ticks = 1;
    out->budget = EMS_MAX_QUOTA;   /* effectively unlimited until 'budget' appears */
    memset(err, 0, sizeof(*err));

    char(*names)[EMS_MAX_NAME] = calloc(EMS_MAX_CURVES, sizeof(*names));
    if (!names) {
        snprintf(err->message, sizeof(err->message), "out of memory");
        return -1;
    }

    Parser ps;
    memset(&ps, 0, sizeof(ps));
    ps.out = out;
    ps.names = names;
    ps.err = err;
    ps.lx.src = src;
    ps.lx.p = src;
    ps.lx.line = 1;
    ps.lx.col = 1;
    ps.lx.last.line = 1;
    ps.lx.last.col = 1;
    if (lex_next(&ps.lx) != 0) {
        fail_at(&ps, ps.lx.tok.line, ps.lx.tok.col, "invalid token");
    } else {
        while (!ps.failed && ps.lx.tok.kind != TK_EOF)
            if (!parse_statement(&ps)) break;
        if (!ps.failed) resolve_refs(&ps);
    }
    free(names);
    return ps.failed ? -1 : 0;
}

/* ---------- execution ---------- */

typedef struct {
    float px, py, vx, vy;
    float age, life;
    int emitter;      /* emitter index while live */
    int next_free;    /* next slot in the free list while dead */
    unsigned char live;
} Slot;

static float ease(EmsShape shape, float u) {
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;
    switch (shape) {
    case EMS_SHAPE_QUAD_IN: return u * u;
    case EMS_SHAPE_QUAD_OUT: return 1.0f - (1.0f - u) * (1.0f - u);
    case EMS_SHAPE_CUBIC_IO:
        return u < 0.5f ? 4.0f * u * u * u : 1.0f - powf(-2.0f * u + 2.0f, 3.0f) / 2.0f;
    case EMS_SHAPE_STEP: return u < 1.0f ? 0.0f : 1.0f;
    case EMS_SHAPE_LINEAR:
    default: return u;
    }
}

static float eval_value(const EmsProgram* prog, const EmsValue* v, float t) {
    if (v->kind == EMS_VALUE_CONST) return v->number;
    const EmsCurve* c = &prog->curves[v->curve];
    if (c->shape == EMS_SHAPE_STEP)
        return t >= 1.0f ? c->to : c->from;
    float phase = t - floorf(t);
    return c->from + (c->to - c->from) * ease(c->shape, phase);
}

int ems_run(const EmsProgram* prog, EmsResult* out) {
    if (!prog || !out) return -1;
    if (prog->budget < 0 || prog->ticks < 1 || prog->ticks > EMS_MAX_TICKS ||
        !(prog->dt >= 0.0f) || !isfinite(prog->dt))
        return -1;

    memset(out, 0, sizeof(*out));
    int ne = prog->emitter_count;
    if (ne == 0) return 0;
    if (prog->pool < 1 || prog->pool > EMS_MAX_POOL) return -1;

    Slot* pool = malloc(sizeof(*pool) * (size_t)prog->pool);
    int* order = malloc(sizeof(int) * (size_t)ne);
    double* carry = calloc((size_t)ne, sizeof(double));
    if (!pool || !order || !carry) {
        free(pool); free(order); free(carry);
        return -1;
    }

    for (int i = 0; i < prog->pool; i++) {
        pool[i].live = 0;
        pool[i].next_free = i + 1 < prog->pool ? i + 1 : -1;
    }
    int free_head = 0;
    int alive = 0;
    int spawned_total[EMS_MAX_EMITTERS] = { 0 };

    /* descending priority; a stable selection sort keeps script order on ties */
    for (int i = 0; i < ne; i++) order[i] = i;
    for (int i = 0; i < ne; i++) {
        int best = i;
        for (int j = i + 1; j < ne; j++)
            if (prog->emitters[order[j]].priority > prog->emitters[order[best]].priority)
                best = j;
        if (best != i) { int t = order[i]; order[i] = order[best]; order[best] = t; }
    }

    for (int tick = 0; tick < prog->ticks; tick++) {
        float t = (float)tick * prog->dt;
        int left = prog->budget;
        for (int k = 0; k < ne && left > 0; k++) {
            int ei = order[k];
            const EmsEmitter* em = &prog->emitters[ei];
            double q = (double)eval_value(prog, &em->quota, t);
            if (!(q >= 0.0)) q = 0.0;
            if (q > (double)left) q = (double)left;
            carry[ei] += q;
            int want = (int)floor(carry[ei]);
            if (want > left) want = left;
            carry[ei] -= (double)want;

            float life = eval_value(prog, &em->life, t);
            if (life < 0.0f) life = 0.0f;
            float px = eval_value(prog, &em->pos[0], t);
            float py = eval_value(prog, &em->pos[1], t);
            float vx = eval_value(prog, &em->vel[0], t);
            float vy = eval_value(prog, &em->vel[1], t);

            int got = 0;
            while (got < want && free_head != -1) {
                int idx = free_head;
                Slot* p = &pool[idx];
                free_head = p->next_free;
                p->live = 1;
                p->px = px; p->py = py;
                p->vx = vx; p->vy = vy;
                p->age = 0.0f;
                p->life = life;
                p->emitter = ei;
                alive++;
                got++;
            }
            spawned_total[ei] += got;
            left -= got;
        }
        if (alive > out->high_water) out->high_water = alive;

        /* age and expire */
        for (int i = 0; i < prog->pool; i++) {
            Slot* p = &pool[i];
            if (!p->live) continue;
            p->age += prog->dt;
            p->px += p->vx * prog->dt;
            p->py += p->vy * prog->dt;
            if (p->age >= p->life) {
                p->live = 0;
                p->next_free = free_head;
                free_head = i;
                alive--;
                out->expired[p->emitter]++;
            }
        }
        out->frame_alive[tick] = alive;
        out->frames = tick + 1;
    }

    for (int i = 0; i < ne; i++) out->spawned[i] = spawned_total[i];
    for (int i = 0; i < prog->pool; i++) {
        const Slot* p = &pool[i];
        if (!p->live) continue;
        out->alive[p->emitter]++;
        out->sum_x += p->px;
        out->sum_y += p->py;
        out->sum_age += p->age;
    }

    free(pool);
    free(order);
    free(carry);
    return 0;
}

/* ---------- diagnostics ---------- */

const char* ems_shape_name(EmsShape shape) {
    switch (shape) {
    case EMS_SHAPE_QUAD_IN: return "quad_in";
    case EMS_SHAPE_QUAD_OUT: return "quad_out";
    case EMS_SHAPE_CUBIC_IO: return "cubic_io";
    case EMS_SHAPE_STEP: return "step";
    case EMS_SHAPE_LINEAR:
    default: return "linear";
    }
}

int ems_format_error(const char* src, const char* filename,
                     const EmsError* err, char* out, size_t cap) {
    if (!src || !err || !out || cap == 0) return 0;
    const char* file = filename ? filename : "<script>";
    int line = err->line > 0 ? err->line : 1;
    int col = err->col > 0 ? err->col : 1;
    int n = snprintf(out, cap, "%s:%d:%d: error: %s\n", file, line, col, err->message);
    if (n < 0 || (size_t)n >= cap) return n;

    const char* p = src;
    for (int l = 1; l < line && *p; p++) if (*p == '\n') l++;
    const char* eol = p;
    while (*eol && *eol != '\n') eol++;
    size_t len = (size_t)(eol - p);
    if (len > 200) len = 200;
    if ((size_t)n + len + (size_t)col + 4 >= cap) return n;

    memcpy(out + n, p, len);
    n += (int)len;
    out[n++] = '\n';
    int caret = col - 1;
    if ((size_t)caret > len) caret = (int)len;
    for (int i = 0; i < caret; i++) out[n++] = (p[i] == '\t') ? '\t' : ' ';
    out[n++] = '^';
    out[n++] = '\n';
    out[n] = '\0';
    return n;
}
