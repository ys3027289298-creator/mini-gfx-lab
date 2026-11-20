/* pf_heap.c - binary min-heap with lexicographic (k1, k2) keys. */
#include <stdlib.h>
#include <string.h>

#include "pf_internal.h"

void pf_heap_init(PfHeap* h) {
    memset(h, 0, sizeof(*h));
}

void pf_heap_free(PfHeap* h) {
    free(h->a);
    memset(h, 0, sizeof(*h));
}

void pf_heap_clear(PfHeap* h) {
    h->n = 0;
}

static int less(const PfHeapNode* a, const PfHeapNode* b) {
    if (a->k1 != b->k1) return a->k1 < b->k1;
    if (a->k2 != b->k2) return a->k2 < b->k2;
    return a->idx < b->idx;
}

static int grow(PfHeap* h) {
    if (h->n < h->cap) return 0;
    int cap = h->cap ? h->cap * 2 : 64;
    PfHeapNode* a = (PfHeapNode*)realloc(h->a, sizeof(*a) * (size_t)cap);
    if (!a) return -1;
    h->a = a;
    h->cap = cap;
    return 0;
}

int pf_heap_push(PfHeap* h, PfHeapNode v) {
    if (grow(h) != 0) return -1;
    int i = h->n++;
    h->a[i] = v;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (!less(&h->a[i], &h->a[p])) break;
        PfHeapNode tmp = h->a[i]; h->a[i] = h->a[p]; h->a[p] = tmp;
        i = p;
    }
    return 0;
}

int pf_heap_pop(PfHeap* h, PfHeapNode* out) {
    if (h->n == 0) return -1;
    *out = h->a[0];
    h->a[0] = h->a[--h->n];
    int i = 0;
    for (;;) {
        int l = 2 * i + 1, r = l + 1, m = i;
        if (l < h->n && less(&h->a[l], &h->a[m])) m = l;
        if (r < h->n && less(&h->a[r], &h->a[m])) m = r;
        if (m == i) break;
        PfHeapNode tmp = h->a[i]; h->a[i] = h->a[m]; h->a[m] = tmp;
        i = m;
    }
    return 0;
}

int pf_heap_peek(PfHeap* h, PfHeapNode* out) {
    if (h->n == 0) return -1;
    *out = h->a[0];
    return 0;
}

/* Drop every entry whose version no longer matches the owner's. Callers that
 * use lazy deletion otherwise accumulate one dead entry per update. */
void pf_heap_compact(PfHeap* h, const unsigned* ver) {
    int m = 0;
    for (int i = 0; i < h->n; i++)
        if (h->a[i].ver == ver[h->a[i].idx]) h->a[m++] = h->a[i];
    h->n = m;
    for (int i = m / 2 - 1; i >= 0; i--) {
        int j = i;
        for (;;) {
            int l = 2 * j + 1, r = l + 1, b = j;
            if (l < h->n && less(&h->a[l], &h->a[b])) b = l;
            if (r < h->n && less(&h->a[r], &h->a[b])) b = r;
            if (b == j) break;
            PfHeapNode tmp = h->a[j]; h->a[j] = h->a[b]; h->a[b] = tmp;
            j = b;
        }
    }
}
