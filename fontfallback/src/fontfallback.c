/* fontfallback: family-priority fallback chain over a set of faces.

   Faces are kept in one chain. A code point is resolved by walking the chain:
   the first face that has the glyph wins, so advance, ink box and side
   bearings all come from that single face - a glyph is never blended from two
   fonts. Every face is normalized to a common 1000-unit em with its baseline
   at y=0, so a glyph picked up from a fallback lands on the same baseline and
   the same advance grid as a glyph from the primary face. A merged single-face
   reference holding the same scaled glyphs therefore lays out identically,
   glyph for glyph. Code points no face covers render as a square tofu and are
   reported as U+XXXX.

   Cost: the requested family ranks matching faces first, then the remaining
   faces in insertion order; the order is built once per chain and cached.
   A lookup walks that order and stops at the hit, so one code point costs at
   most `face_count` probes. The probe counters make that linear bound
   testable instead of a wall-clock measurement. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_FACES 8
#define MAX_GLYPHS 32
#define MAX_KERN 16
#define MAX_PLACED 64
#define BASE_UPEM 1000
#define TOFU_ADVANCE 700
#define TOFU_BOX 600

typedef struct {
    unsigned cp;
    int advance;
    int ink_w, ink_h;
    int bx, by;        /* ink left, ink top above the baseline */
} GlyphDef;            /* in font units, +y up */

typedef struct {
    unsigned left, right;
    int delta;         /* font units in FontSpec, pre-scaled in Face */
} Kern;

typedef struct {
    const char* name;
    const char* families;   /* space-separated aliases, e.g. "Sans Helvetica" */
    int upem;
    int ascent, descent;    /* font units above/below the baseline */
    const GlyphDef* glyphs;
    int glyph_count;
    const Kern* kerns;
    int kern_count;
} FontSpec;

typedef struct {
    unsigned cp;
    int advance;
    int ink_w, ink_h;
    int bx, by;        /* BASE_UPEM units, baseline at y=0 */
} SGlyph;

typedef struct {
    const char* name;
    const char* families;
    int ascent, descent;            /* BASE_UPEM units */
    int glyph_count;
    int kern_count;
    SGlyph glyphs[MAX_GLYPHS];      /* sorted by cp */
    Kern kerns[MAX_KERN];
    unsigned probes;                /* faces examined by chain lookups */
} Face;

typedef struct {
    Face faces[MAX_FACES];
    int count;
    int order[MAX_FACES];           /* cached lookup order */
    int order_count;
    unsigned order_builds;
} Chain;

typedef struct {
    int face;                       /* index into Chain.faces, -1 = tofu */
    unsigned cp;
    int advance;
    int ink_w, ink_h;
    int bx, by;
} Resolved;

typedef struct {
    unsigned cp;
    int face;
    int x;                          /* pen x at the glyph origin */
    int advance;
    int ink_w, ink_h;
    int bx, by;                     /* baseline-relative, +y up */
} Placed;

typedef struct {
    Placed glyphs[MAX_PLACED];
    int count;
    int ascent, descent;            /* line box, BASE_UPEM units */
    unsigned missing[MAX_PLACED];
    int missing_count;
    int bad_bytes;
} Line;

/* ---------------- scaling and lookup ---------------- */

/* font units -> BASE_UPEM, rounded half away from zero */
static int scale_units(int v, int upem) {
    long num = (long)v * BASE_UPEM;
    long rem = num % upem;
    long q = num / upem;
    if (rem < 0) rem = -rem;
    if (rem * 2 >= upem) q += (num < 0 ? -1 : 1);
    return (int)q;
}

static int cmp_glyph(const void* a, const void* b) {
    unsigned x = ((const SGlyph*)a)->cp, y = ((const SGlyph*)b)->cp;
    return x < y ? -1 : (x > y ? 1 : 0);
}

static int family_match(const char* families, const char* want) {
    size_t wl = strlen(want);
    for (const char* p = families; *p; ) {
        while (*p == ' ') p++;
        const char* q = p;
        while (*q && *q != ' ') q++;
        if ((size_t)(q - p) == wl && !strncmp(p, want, wl)) return 1;
        p = q;
    }
    return 0;
}

/* matching families first, then everyone else, both in insertion order */
static void build_order(Chain* ch, const char* family) {
    unsigned char picked[MAX_FACES] = { 0 };
    int n = 0;
    if (family && family[0]) {
        for (int i = 0; i < ch->count; i++)
            if (family_match(ch->faces[i].families, family)) {
                ch->order[n++] = i;
                picked[i] = 1;
            }
    }
    for (int i = 0; i < ch->count; i++)
        if (!picked[i]) ch->order[n++] = i;
    ch->order_count = n;
    ch->order_builds++;
}

static int chain_init(Chain* ch, const FontSpec* specs, int n, const char* family) {
    if (n <= 0 || n > MAX_FACES) return -1;
    memset(ch, 0, sizeof(*ch));
    for (int i = 0; i < n; i++) {
        const FontSpec* s = &specs[i];
        if (!s->name || !s->families || s->upem <= 0) return -1;
        if (s->glyph_count < 0 || s->glyph_count > MAX_GLYPHS) return -1;
        if (s->kern_count < 0 || s->kern_count > MAX_KERN) return -1;
        if (s->glyph_count && !s->glyphs) return -1;
        if (s->kern_count && !s->kerns) return -1;

        Face* f = &ch->faces[i];
        f->name = s->name;
        f->families = s->families;
        f->ascent = scale_units(s->ascent, s->upem);
        f->descent = scale_units(s->descent, s->upem);
        for (int g = 0; g < s->glyph_count; g++) {
            const GlyphDef* d = &s->glyphs[g];
            SGlyph* out = &f->glyphs[g];
            out->cp = d->cp;
            out->advance = scale_units(d->advance, s->upem);
            out->ink_w = scale_units(d->ink_w, s->upem);
            out->ink_h = scale_units(d->ink_h, s->upem);
            out->bx = scale_units(d->bx, s->upem);
            out->by = scale_units(d->by, s->upem);
        }
        f->glyph_count = s->glyph_count;
        qsort(f->glyphs, (size_t)f->glyph_count, sizeof(SGlyph), cmp_glyph);
        for (int g = 1; g < f->glyph_count; g++)
            if (f->glyphs[g].cp == f->glyphs[g - 1].cp) return -1;
        for (int k = 0; k < s->kern_count; k++) {
            f->kerns[k].left = s->kerns[k].left;
            f->kerns[k].right = s->kerns[k].right;
            f->kerns[k].delta = scale_units(s->kerns[k].delta, s->upem);
        }
        f->kern_count = s->kern_count;
    }
    ch->count = n;
    build_order(ch, family);
    return 0;
}

static int face_glyph(const Face* f, unsigned cp) {
    int lo = 0, hi = f->glyph_count - 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (f->glyphs[mid].cp == cp) return mid;
        if (f->glyphs[mid].cp < cp) lo = mid + 1;
        else hi = mid - 1;
    }
    return -1;
}

static int face_kern(const Face* f, unsigned left, unsigned right) {
    for (int i = 0; i < f->kern_count; i++)
        if (f->kerns[i].left == left && f->kerns[i].right == right)
            return f->kerns[i].delta;
    return 0;
}

/* 1 when a face covered cp; 0 means the caller draws tofu */
static int chain_resolve(Chain* ch, unsigned cp, Resolved* out) {
    memset(out, 0, sizeof(*out));
    out->cp = cp;
    out->face = -1;
    for (int i = 0; i < ch->order_count; i++) {
        Face* f = &ch->faces[ch->order[i]];
        f->probes++;
        int gi = face_glyph(f, cp);
        if (gi < 0) continue;
        const SGlyph* g = &f->glyphs[gi];
        out->face = ch->order[i];
        out->advance = g->advance;
        out->ink_w = g->ink_w;
        out->ink_h = g->ink_h;
        out->bx = g->bx;
        out->by = g->by;
        return 1;
    }
    /* square tofu sitting on the baseline */
    out->advance = TOFU_ADVANCE;
    out->ink_w = out->ink_h = TOFU_BOX;
    out->bx = (TOFU_ADVANCE - TOFU_BOX) / 2;
    out->by = TOFU_BOX;
    return 0;
}

/* ---------------- shaping ---------------- */

static int utf8_decode(const char* s, unsigned* cp, int* len) {
    const unsigned char* p = (const unsigned char*)s;
    int n;
    unsigned v;
    if (p[0] < 0x80) { *cp = p[0]; *len = 1; return 1; }
    else if ((p[0] & 0xE0) == 0xC0) { n = 2; v = p[0] & 0x1Fu; }
    else if ((p[0] & 0xF0) == 0xE0) { n = 3; v = p[0] & 0x0Fu; }
    else if ((p[0] & 0xF8) == 0xF0) { n = 4; v = p[0] & 0x07u; }
    else return 0;
    for (int i = 1; i < n; i++) {
        if ((p[i] & 0xC0) != 0x80) return 0;
        v = (v << 6) | (p[i] & 0x3Fu);
    }
    *cp = v;
    *len = n;
    return 1;
}

static void line_add_missing(Line* line, unsigned cp) {
    for (int i = 0; i < line->missing_count; i++)
        if (line->missing[i] == cp) return;
    if (line->missing_count < MAX_PLACED)
        line->missing[line->missing_count++] = cp;
}

static void line_shape(Chain* ch, const char* text, Line* line) {
    memset(line, 0, sizeof(*line));
    int pen = 0, n = 0;
    for (const char* p = text; *p && n < MAX_PLACED; ) {
        unsigned cp;
        int len;
        if (!utf8_decode(p, &cp, &len)) {
            line->bad_bytes++;
            p++;
            continue;
        }
        Resolved r;
        int covered = chain_resolve(ch, cp, &r);
        Placed* g = &line->glyphs[n];
        int x = pen;
        /* kerning only within one face: a cross-face pair has no owner */
        if (n > 0 && r.face >= 0 && r.face == line->glyphs[n - 1].face)
            x += face_kern(&ch->faces[r.face], line->glyphs[n - 1].cp, cp);
        g->cp = cp;
        g->face = r.face;
        g->x = x;
        g->advance = r.advance;
        g->ink_w = r.ink_w;
        g->ink_h = r.ink_h;
        g->bx = r.bx;
        g->by = r.by;
        pen = x + r.advance;
        if (!covered) line_add_missing(line, cp);
        n++;
        p += len;
    }
    line->count = n;

    /* line box: every face that supplied a glyph; tofu counts as the primary */
    int asc = -1, desc = 0;
    for (int i = 0; i < line->count; i++) {
        int fi = line->glyphs[i].face;
        if (fi < 0) fi = ch->order[0];
        if (ch->faces[fi].ascent > asc) asc = ch->faces[fi].ascent;
        if (ch->faces[fi].descent > desc) desc = ch->faces[fi].descent;
    }
    if (asc < 0) {
        asc = ch->faces[ch->order[0]].ascent;
        desc = ch->faces[ch->order[0]].descent;
    }
    line->ascent = asc;
    line->descent = desc;
}

static int line_width(const Line* line) {
    if (line->count == 0) return 0;
    return line->glyphs[line->count - 1].x + line->glyphs[line->count - 1].advance;
}

static void line_missing_report(const Line* line, char* buf, size_t cap) {
    size_t off = 0;
    if (cap == 0) return;
    buf[0] = 0;
    for (int i = 0; i < line->missing_count; i++) {
        int w = snprintf(buf + off, cap - off, "%sU+%04X", i ? "," : "", line->missing[i]);
        if (w < 0) break;
        off += (size_t)w;
        if (off >= cap) { buf[cap - 1] = 0; break; }
    }
}

/* ---------------- self-check ---------------- */

/* One full-coverage face equivalent to laying src[0..n) over each other.
   Requires disjoint code point coverage and no cross-face kern pairs. */
static int merge_faces(const FontSpec* src, int n, FontSpec* ref,
                       GlyphDef* glyphs, int glyph_cap,
                       Kern* kerns, int kern_cap) {
    int gc = 0, kc = 0, asc = 0, desc = 0;
    for (int i = 0; i < n; i++) {
        const FontSpec* s = &src[i];
        int sa = scale_units(s->ascent, s->upem);
        int sd = scale_units(s->descent, s->upem);
        if (sa > asc) asc = sa;
        if (sd > desc) desc = sd;
        for (int g = 0; g < s->glyph_count; g++) {
            if (gc >= glyph_cap) return -1;
            const GlyphDef* d = &s->glyphs[g];
            GlyphDef m;
            m.cp = d->cp;
            m.advance = scale_units(d->advance, s->upem);
            m.ink_w = scale_units(d->ink_w, s->upem);
            m.ink_h = scale_units(d->ink_h, s->upem);
            m.bx = scale_units(d->bx, s->upem);
            m.by = scale_units(d->by, s->upem);
            glyphs[gc++] = m;
        }
        for (int k = 0; k < s->kern_count; k++) {
            if (kc >= kern_cap) return -1;
            kerns[kc].left = s->kerns[k].left;
            kerns[kc].right = s->kerns[k].right;
            kerns[kc].delta = scale_units(s->kerns[k].delta, s->upem);
            kc++;
        }
    }
    ref->name = "Ref";
    ref->families = "Ref";
    ref->upem = BASE_UPEM;
    ref->ascent = asc;
    ref->descent = desc;
    ref->glyphs = glyphs;
    ref->glyph_count = gc;
    ref->kerns = kerns;
    ref->kern_count = kc;
    return 0;
}

static unsigned total_probes(const Chain* ch) {
    unsigned n = 0;
    for (int i = 0; i < ch->count; i++) n += ch->faces[i].probes;
    return n;
}

static void reset_probes(Chain* ch) {
    for (int i = 0; i < ch->count; i++) ch->faces[i].probes = 0;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Chain ch;
    Line line, ref_line;
    Resolved r;
    char report[128];

    /* three faces at different upems and baselines, disjoint coverage */
    static const GlyphDef latin_glyphs[] = {
        { 'A', 600, 560, 700,  20, 700 },
        { 'V', 600, 540, 700,  30, 700 },
        { 'T', 580, 520, 700,  30, 700 },
        { 'o', 550, 500, 480,  30, 480 },
    };
    static const Kern latin_kerns[] = { { 'A', 'V', -80 }, { 'T', 'o', -60 } };
    static const GlyphDef cjk_glyphs[] = {
        { 0x6C49, 2048, 1900, 1900, 74, 1900 },   /* 汉 */
        { 0x5B57, 2048, 1900, 1900, 74, 1900 },   /* 字 */
    };
    static const GlyphDef sym_glyphs[] = {
        { 0x1F600, 512, 480, 480, 16, 480 },      /* emoji */
    };
    static const FontSpec latin = { "Latin", "Latin Sans", 1000, 800, 200,
                                    latin_glyphs, 4, latin_kerns, 2 };
    static const FontSpec cjk   = { "CJK", "CJK Han", 2048, 1800, 248,
                                    cjk_glyphs, 2, NULL, 0 };
    static const FontSpec sym   = { "Sym", "Symbols Emoji", 512, 480, 32,
                                    sym_glyphs, 1, NULL, 0 };
    const FontSpec specs[] = { latin, cjk, sym };

    check("chain_init", chain_init(&ch, specs, 3, NULL) == 0);
    check("face_order_default", ch.order[0] == 0 && ch.order[1] == 1 && ch.order[2] == 2);

    /* primary face */
    check("primary_hit", chain_resolve(&ch, 'A', &r) == 1 && r.face == 0 && r.advance == 600);

    /* 2048-upem face scaled onto the 1000-unit grid */
    check("fallback_level2", chain_resolve(&ch, 0x6C49, &r) == 1 && r.face == 1);
    check("fallback_scaled_advance", r.advance == 1000);
    check("fallback_scaled_ink", r.ink_w == 928 && r.ink_h == 928 && r.by == 928);
    check("fallback_scaled_bearing", r.bx == 36);

    /* 512-upem face: 480 units is exactly half a pixel unit, rounds away */
    check("fallback_level3", chain_resolve(&ch, 0x1F600, &r) == 1 && r.face == 2);
    check("scale_half_rounds_up", r.advance == 1000 && r.ink_w == 938 && r.by == 938);
    check("scale_round_helpers", scale_units(480, 512) == 938 && scale_units(32, 512) == 63);

    /* line layout: faces with different ascents share one baseline */
    const char* text = "AVTo\xE6\xB1\x89\xF0\x9F\x98\x80";   /* AVTo 汉 emoji */
    line_shape(&ch, text, &line);
    check("line_count", line.count == 6);
    check("line_ascent_max", line.ascent == 938);             /* sym 480/512 */
    check("line_descent_max", line.descent == 200);           /* latin */
    check("fallback_advance_aligned", line.glyphs[4].advance == 1000 &&
          line.glyphs[5].x == line.glyphs[4].x + 1000);
    check("kern_primary", line.glyphs[1].x == 520 && line.glyphs[1].advance == 600);
    /* 600 + 600 + 580 + 550 + 1000 + 1000, kerns A-V and T-o inside Latin */
    check("line_width", line_width(&line) == 4190);
    check("line_no_missing", line.missing_count == 0);

    /* same text through one full-coverage face must match glyph for glyph */
    FontSpec ref_spec;
    GlyphDef ref_glyphs[64];
    Kern ref_kerns[32];
    check("merge_faces", merge_faces(specs, 3, &ref_spec, ref_glyphs, 64,
                                     ref_kerns, 32) == 0);
    Chain ref_ch;
    check("ref_chain_init", chain_init(&ref_ch, &ref_spec, 1, "Ref") == 0);
    line_shape(&ref_ch, text, &ref_line);

    int same_count = line.count == ref_line.count;
    int same_x = same_count, same_adv = same_count, same_ink = same_count;
    for (int i = 0; i < line.count && i < ref_line.count; i++) {
        same_x   &= line.glyphs[i].x == ref_line.glyphs[i].x;
        same_adv &= line.glyphs[i].advance == ref_line.glyphs[i].advance;
        same_ink &= line.glyphs[i].ink_w == ref_line.glyphs[i].ink_w &&
                    line.glyphs[i].ink_h == ref_line.glyphs[i].ink_h &&
                    line.glyphs[i].bx == ref_line.glyphs[i].bx &&
                    line.glyphs[i].by == ref_line.glyphs[i].by;
    }
    check("ref_count", same_count);
    check("ref_positions", same_x);
    check("ref_advances", same_adv);
    check("ref_ink_baseline", same_ink);
    check("ref_line_box", line.ascent == ref_line.ascent &&
          line.descent == ref_line.descent);
    check("ref_width", line_width(&line) == line_width(&ref_line));

    /* empty line uses the primary face box */
    line_shape(&ch, "", &line);
    check("empty_line", line.count == 0 && line.ascent == 800 && line.descent == 200);

    /* family priority: matching faces lead the chain, aliases included */
    static const GlyphDef sans_glyph[] = { { 'Z', 600, 500, 700, 20, 700 } };
    static const GlyphDef serif_glyph[] = { { 'Z', 610, 520, 700, 20, 700 } };
    static const FontSpec sans = { "Sans", "Sans", 1000, 800, 200,
                                   sans_glyph, 1, NULL, 0 };
    static const FontSpec serif = { "Serif", "Serif SerifAlias", 1000, 820, 180,
                                    serif_glyph, 1, NULL, 0 };
    const FontSpec duo[] = { sans, serif };
    Chain duo_ch;
    check("priority_chain", chain_init(&duo_ch, duo, 2, "SerifAlias") == 0);
    check("priority_family_first", duo_ch.order[0] == 1 && duo_ch.order[1] == 0);
    reset_probes(&duo_ch);
    check("priority_family_wins", chain_resolve(&duo_ch, 'Z', &r) == 1 &&
          r.face == 1 && r.advance == 610);
    check("priority_probes", duo_ch.faces[1].probes == 1 && duo_ch.faces[0].probes == 0);
    check("priority_default_order", chain_init(&duo_ch, duo, 2, "Nope") == 0 &&
          duo_ch.order[0] == 0);
    check("priority_default_wins", chain_resolve(&duo_ch, 'Z', &r) == 1 &&
          r.face == 0 && r.advance == 600);

    /* missing code points: tofu plus a U+XXXX report, layout keeps going */
    line_shape(&ch, "A\xE4\xB8\xADV", &line);   /* A 中(4E2D) V */
    check("tofu_count", line.count == 3);
    check("tofu_face", line.glyphs[1].face == -1);
    check("tofu_square", line.glyphs[1].ink_w == TOFU_BOX &&
          line.glyphs[1].ink_h == TOFU_BOX && line.glyphs[1].by == TOFU_BOX);
    check("tofu_advances", line.glyphs[2].x == 600 + TOFU_ADVANCE);
    check("tofu_primary_box", line.ascent == 800 && line.descent == 200);
    line_missing_report(&line, report, sizeof(report));
    check("missing_report", line.missing_count == 1 && !strcmp(report, "U+4E2D"));

    line_shape(&ch, "\xE4\xB8\xAD\xE6\x96\x87", &line);   /* 中文 */
    line_missing_report(&line, report, sizeof(report));
    check("missing_report_multi", line.missing_count == 2 &&
          !strcmp(report, "U+4E2D,U+6587"));
    check("missing_still_advances", line.count == 2 &&
          line.glyphs[1].x == TOFU_ADVANCE);

    line_shape(&ch, "\xE4\xB8\xAD\xE4\xB8\xAD", &line);   /* 中中: reported once */
    line_missing_report(&line, report, sizeof(report));
    check("missing_report_dedup", line.missing_count == 1 &&
          !strcmp(report, "U+4E2D"));

    /* kerning never crosses faces */
    static const GlyphDef x_glyphs[] = { { 'X', 600, 500, 700, 20, 700 } };
    static const Kern x_kerns[] = { { 'X', 'V', -70 } };
    static const GlyphDef v_glyphs[] = { { 'V', 500, 500, 700, 20, 700 } };
    static const FontSpec xf = { "Xf", "Xf", 1000, 800, 200, x_glyphs, 1, x_kerns, 1 };
    static const FontSpec vf = { "Vf", "Vf", 1000, 800, 200, v_glyphs, 1, NULL, 0 };
    const FontSpec cross[] = { xf, vf };
    Chain cross_ch;
    check("cross_chain", chain_init(&cross_ch, cross, 2, NULL) == 0);
    line_shape(&cross_ch, "XV", &line);
    check("kern_same_face_off", line.glyphs[1].face == 1 && line.glyphs[1].x == 600);

    /* probe count is linear in the face count and the order is built once */
    static const GlyphDef probe_glyphs[8] = {
        { 'a', 500, 0, 0, 0, 0 }, { 'b', 500, 0, 0, 0, 0 },
        { 'c', 500, 0, 0, 0, 0 }, { 'd', 500, 0, 0, 0, 0 },
        { 'e', 500, 0, 0, 0, 0 }, { 'f', 500, 0, 0, 0, 0 },
        { 'g', 500, 0, 0, 0, 0 }, { 'h', 500, 0, 0, 0, 0 },
    };
    static const char* const probe_names[8] = {
        "P0", "P1", "P2", "P3", "P4", "P5", "P6", "P7"
    };
    FontSpec probe_specs[8];
    for (int i = 0; i < 8; i++)
        probe_specs[i] = (FontSpec){ probe_names[i], "", 1000, 800, 200,
                                     &probe_glyphs[i], 1, NULL, 0 };

    Chain p8;
    check("probe_chain", chain_init(&p8, probe_specs, 8, NULL) == 0);
    reset_probes(&p8);
    check("probe_first_is_one", chain_resolve(&p8, 'a', &r) == 1 &&
          r.face == 0 && total_probes(&p8) == 1);
    reset_probes(&p8);
    check("probe_last_is_eight", chain_resolve(&p8, 'h', &r) == 1 &&
          r.face == 7 && total_probes(&p8) == 8);
    reset_probes(&p8);
    Line probe_line;
    line_shape(&p8, "abcdefgh", &probe_line);
    check("probe_line_linear", total_probes(&p8) == 36 &&
          total_probes(&p8) <= (unsigned)8 * 8);
    check("probe_order_cached", p8.order_builds == 1);

    Chain p4;
    check("probe_chain4", chain_init(&p4, probe_specs, 4, NULL) == 0);
    reset_probes(&p4);
    check("probe_growth_linear", chain_resolve(&p4, 'd', &r) == 1 &&
          total_probes(&p4) == 4);

    /* malformed input */
    line_shape(&ch, "\xFF" "A", &line);
    check("bad_utf8_skipped", line.count == 1 && line.bad_bytes == 1 &&
          line.glyphs[0].cp == 'A');

    FontSpec bad_spec = latin;
    bad_spec.upem = 0;
    Chain bad;
    check("reject_zero_upem", chain_init(&bad, &bad_spec, 1, NULL) == -1);

    FontSpec too_many[9];
    for (int i = 0; i < 9; i++) too_many[i] = probe_specs[i % 8];
    check("reject_too_many_faces", chain_init(&bad, too_many, 9, NULL) == -1);

    static const GlyphDef dup_glyphs[] = {
        { 'a', 500, 0, 0, 0, 0 }, { 'a', 500, 0, 0, 0, 0 }
    };
    FontSpec dup = { "Dup", "Dup", 1000, 800, 200, dup_glyphs, 2, NULL, 0 };
    check("reject_duplicate_cp", chain_init(&bad, &dup, 1, NULL) == -1);

    printf("missing=%s\n", report);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
