/* ems-repl: load an emitter script at run time, or re-read it on demand.

   Usage:
       ems-repl <file>              compile, run, print a digest
       ems-repl <file> --watch      same, then wait for a newline on stdin and
                                    re-read/re-run until EOF (tweak the script,
                                    press Enter, see the new numbers)

   Exit code: 0 on success, 1 when the script fails to compile or run. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ems.h"

static char* read_file(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return NULL;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
    long n = ftell(f);
    if (n < 0) { fclose(f); return NULL; }
    rewind(f);
    char* buf = malloc((size_t)n + 1);
    if (!buf) { fclose(f); return NULL; }
    size_t got = fread(buf, 1, (size_t)n, f);
    buf[got] = '\0';
    fclose(f);
    return buf;
}

static int load_and_run(const char* path) {
    char* src = read_file(path);
    if (!src) {
        fprintf(stderr, "%s: cannot open\n", path);
        return 1;
    }
    EmsProgram prog;
    EmsError err;
    if (ems_compile(src, &prog, &err) != 0) {
        char msg[1024];
        ems_format_error(src, path, &err, msg, sizeof(msg));
        fputs(msg, stderr);
        free(src);
        return 1;
    }
    EmsResult res;
    if (ems_run(&prog, &res) != 0) {
        fprintf(stderr, "%s: program cannot run (invalid limits)\n", path);
        free(src);
        return 1;
    }
    printf("%s: emitters=%d curves=%d pool=%d budget=%d ticks=%d dt=%.3f\n",
           path, prog.emitter_count, prog.curve_count, prog.pool, prog.budget,
           prog.ticks, (double)prog.dt);
    printf("high_water=%d frames=%d\n", res.high_water, res.frames);
    for (int i = 0; i < prog.emitter_count; i++)
        printf("  %-12s spawned=%-6d expired=%-6d alive=%-6d\n",
               prog.emitters[i].name, res.spawned[i], res.expired[i], res.alive[i]);
    printf("sum=(%.3f, %.3f, %.3f)\n", res.sum_x, res.sum_y, res.sum_age);
    free(src);
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s <script.emitters> [--watch]\n", argv[0]);
        return 2;
    }
    int rc = load_and_run(argv[1]);
    if (rc != 0) return rc;
    if (argc < 3 || strcmp(argv[2], "--watch") != 0) return 0;
    char line[256];
    while (fgets(line, sizeof(line), stdin)) {
        printf("--- reload ---\n");
        rc = load_and_run(argv[1]);
        if (rc != 0) rc = 0;   /* a broken edit should not end the watch */
    }
    return rc;
}
