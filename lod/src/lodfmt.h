/* lodfmt: the one on-disk format shared by the offline baker and the runtime
   selector.  A pack is a header, a mesh table, a level table and raw vertex /
   index blobs.  Every integer is little-endian; every field is written field
   by field so the layout does not depend on host struct padding. */
#ifndef LODFMT_H
#define LODFMT_H

#include <stddef.h>
#include <stdint.h>

#define LOD_FORMAT_MAGIC "LODPACK1"
#define LOD_FORMAT_VERSION 1u
#define LOD_HEADER_BYTES 64u
#define LOD_MESH_REC_BYTES 64u
#define LOD_LEVEL_REC_BYTES 64u
#define LOD_MAX_MESHES 64u
#define LOD_MAX_LEVELS_PER_MESH 16u

/* header field offsets */
#define LOD_HDR_OFF_MAGIC 0u
#define LOD_HDR_OFF_VERSION 8u
#define LOD_HDR_OFF_HEADER_BYTES 12u
#define LOD_HDR_OFF_MESH_COUNT 16u
#define LOD_HDR_OFF_LEVEL_COUNT 20u
#define LOD_HDR_OFF_TABLE_OFF 24u
#define LOD_HDR_OFF_TABLE_BYTES 32u
#define LOD_HDR_OFF_DATA_OFF 40u
#define LOD_HDR_OFF_FILE_BYTES 48u
#define LOD_HDR_OFF_PAYLOAD_CRC 56u
#define LOD_HDR_OFF_FLAGS 60u

typedef enum {
    LOD_OK = 0,
    LOD_ERR_IO,
    LOD_ERR_SIZE,
    LOD_ERR_MAGIC,
    LOD_ERR_VERSION,
    LOD_ERR_HEADER,
    LOD_ERR_TABLE,
    LOD_ERR_CRC,
    LOD_ERR_MESH,
    LOD_ERR_LEVEL,
    LOD_ERR_COUNTS,
    LOD_ERR_RANGE,
    LOD_ERR_BBOX,
    LOD_ERR_MONOTONE,
    LOD_ERR_INDEX
} LodError;

const char* lod_error_str(LodError e);

typedef struct { float min[3]; float max[3]; } LodBBox;

void lod_bbox_init(LodBBox* b);
void lod_bbox_add(LodBBox* b, float x, float y, float z);
int lod_bbox_is_valid(const LodBBox* b);
int lod_bbox_contains(const LodBBox* outer, const LodBBox* inner);
LodBBox lod_bbox_union(const LodBBox* a, const LodBBox* b);

/* raw little-endian helpers, also used by the tamper tests */
uint32_t lod_get_u32(const uint8_t* p);
uint64_t lod_get_u64(const uint8_t* p);
float lod_get_f32(const uint8_t* p);
void lod_put_u32(uint8_t* p, uint32_t v);
void lod_put_u64(uint8_t* p, uint64_t v);
void lod_put_f32(uint8_t* p, float v);
uint32_t lod_crc32(const void* data, size_t len);

/* metadata as seen by the runtime (decoded from the tables) */
typedef struct {
    uint32_t mesh_id;
    uint32_t level_count;
    uint32_t source_vert_count;
    uint32_t source_tri_count;
    LodBBox source_bounds;
    uint64_t levels_off;
} LodMeshView;

typedef struct {
    uint32_t level;
    uint32_t vertex_count;
    uint32_t tri_count;
    float geom_error;
    LodBBox bounds;
    uint64_t verts_off;
    uint64_t idx_off;
    uint32_t verts_bytes;
    uint32_t idx_bytes;
} LodLevelView;

typedef struct {
    uint32_t version;
    uint32_t mesh_count;
    uint32_t level_count;
    uint64_t table_off;
    uint64_t table_bytes;
    uint64_t data_off;
    uint64_t file_bytes;
    uint32_t payload_crc;
    uint32_t flags;
} LodHeader;

typedef struct {
    uint8_t* buf;
    size_t size;
    LodHeader hdr;
} LodPack;

/* offline side: one level of one mesh */
typedef struct {
    uint32_t level;
    uint32_t vertex_count;
    uint32_t tri_count;
    float geom_error;
    LodBBox bounds;          /* must contain source_bounds */
    const float* verts;      /* 3 * vertex_count, xyz */
    const uint32_t* indices; /* 3 * tri_count */
} LodLevelInput;

typedef struct {
    uint32_t mesh_id;
    uint32_t level_count;
    uint32_t source_vert_count;
    uint32_t source_tri_count;
    LodBBox source_bounds;
    const LodLevelInput* levels;
} LodMeshInput;

/* Validate + write a pack.  The writer validates its own bytes before doing
   I/O, so a bad bake cannot produce a file the runtime would reject. */
LodError lod_pack_write(const char* path, const LodMeshInput* meshes,
                        uint32_t mesh_count, uint64_t* out_bytes,
                        char* err, size_t errlen);

/* full structural validation of an in-memory pack */
LodError lod_pack_validate(const uint8_t* buf, size_t size,
                           uint32_t* bad_mesh, uint32_t* bad_level);

LodError lod_read_header(const uint8_t* buf, size_t size, LodHeader* out);

/* runtime side */
LodError lod_pack_load(const char* path, LodPack* out, char* err, size_t errlen);
void lod_pack_free(LodPack* p);
int lod_pack_find_mesh(const LodPack* p, uint32_t mesh_id);
int lod_pack_mesh(const LodPack* p, uint32_t mesh_index, LodMeshView* out);
int lod_pack_level(const LodPack* p, uint32_t mesh_index, uint32_t level_index,
                   LodLevelView* out);

#endif
