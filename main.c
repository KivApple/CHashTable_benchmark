#include <assert.h>
#include <stdio.h>
#include <CEssentials/dynvec.h>
#include <CEssentials/hashtable.h>
#include <cglm/vec3.h>
#include <cglm/ivec3.h>
#ifdef _WIN32
#include <windows.h>
#endif
#ifndef _MSC_VER
#include <pthread.h>
#include <sys/time.h>
#endif

typedef dynvec(vec3) Vec3List;
typedef dynvec(ivec3) IVec3List;

typedef struct EdgeId {
	int first;
	int second;
} EdgeId;

static inline size_t EdgeId_hash(EdgeId x) {
	return ht_hash_combine(ht_int_hash(x.first), ht_int_hash(x.second));
}

static inline bool EdgeId_eq(EdgeId a, EdgeId b) {
	return a.first == b.first && a.second == b.second;
}

typedef HT(EdgeId, int) MeshEdgeCache;

typedef struct IndexedMesh {
	Vec3List vertices;
	IVec3List triangles;
} IndexedMesh;

static int midVertexForEdge(MeshEdgeCache *cache, Vec3List *vertices, int first, int second) {
	if (first > second) {
		int tmp = first;
		first = second;
		second = tmp;
	}
	EdgeId key = {first, second};
	int result;
	size_t i = 0;
	ht_put(*cache, EdgeId, int, key, i, result, EdgeId_hash, EdgeId_eq);
	if (result) {
		ht_value(*cache, i) = (int) dynvec_size(*vertices);
		vec3 *v = dynvec_append(*vertices, vec3);
		glm_vec3_add(dynvec_at(*vertices, first), dynvec_at(*vertices, second), *v);
		glm_vec3_normalize(*v);
	}
	return ht_value(*cache, i);
}

static void subdivideMesh(Vec3List *vertices, IVec3List *triangles, MeshEdgeCache *cache, IVec3List *result) {
	ht_clear(*cache);
	dynvec_clear(*result);
	dynvec_for_each(*triangles, i) {
		ivec3 *triangle = &dynvec_at(*triangles, i);
		int mid[] = {
				midVertexForEdge(cache, vertices, (*triangle)[0], (*triangle)[1]),
				midVertexForEdge(cache, vertices, (*triangle)[1], (*triangle)[2]),
				midVertexForEdge(cache, vertices, (*triangle)[2], (*triangle)[0])
		};
		glm_ivec3_copy((ivec3) {(*triangle)[0], mid[0], mid[2]}, *dynvec_append(*result, ivec3));
		glm_ivec3_copy((ivec3) {(*triangle)[1], mid[1], mid[0]}, *dynvec_append(*result, ivec3));
		glm_ivec3_copy((ivec3) {(*triangle)[2], mid[2], mid[1]}, *dynvec_append(*result, ivec3));
		glm_ivec3_copy((ivec3) {mid[0], mid[1], mid[2]}, *dynvec_append(*result, ivec3));
	}
	assert(dynvec_size(*result) == dynvec_size(*triangles) * 4);
	assert(ht_size(*cache) == dynvec_size(*triangles) + dynvec_size(*triangles) / 2);
}

IndexedMesh generateMesh(int subdivisionCount) {
#define X 0.525731112119133606f
#define Z 0.850650808352039932f
#define N 0.0f
	static const vec3 INITIAL_VERTICES[] = {
			{-X, N, Z}, {X, N,Z}, {-X, N, -Z}, {X, N, -Z},
			{N, Z, X}, {N, Z,-X}, {N,-Z,X}, {N,-Z,-X},
			{Z, X, N}, {-Z, X, N}, {Z,-X,N}, {-Z,-X, N}
	};
#undef N
#undef Z
#undef X
	
	static const ivec3 INITIAL_TRIANGLES[] = {
			{0, 4, 1}, {0, 9, 4}, {9, 5, 4}, {4, 5, 8}, {4, 8, 1},
			{8, 10, 1}, {8, 3, 10}, {5, 3, 8}, {5, 2, 3}, {2, 7, 3},
			{7, 10, 3}, {7, 6, 10}, {7, 11, 6}, {11, 0, 6}, {0, 1, 6},
			{6, 1, 10}, {9, 0, 11}, {9, 11, 2}, {9, 2, 5}, {7, 2, 11}
	};
	
	Vec3List vertices;
	dynvec_init(vertices);
	dynvec_resize(vertices, sizeof(INITIAL_VERTICES) / sizeof(INITIAL_VERTICES[0]), vec3);
	memcpy(&dynvec_at(vertices, 0), INITIAL_VERTICES, sizeof(INITIAL_VERTICES));
	
	IVec3List trianglesStorage[2];
	IVec3List *triangles = &trianglesStorage[0];
	IVec3List *tmpTriangles = &trianglesStorage[1];
	
	dynvec_init(*triangles);
	dynvec_resize(*triangles, sizeof(INITIAL_TRIANGLES) / sizeof(INITIAL_TRIANGLES[0]), ivec3);
	memcpy(&dynvec_at(*triangles, 0), INITIAL_TRIANGLES, sizeof(INITIAL_TRIANGLES));
	
	dynvec_init(*tmpTriangles);
	
	MeshEdgeCache cache;
	ht_init(cache);
	
	size_t predictedVertexCount = dynvec_size(vertices);
	size_t predictedTriangleCount = dynvec_size(*triangles);
	size_t predictedCacheSize = 0;
	for (int i = 0; i < subdivisionCount; i++) {
		predictedCacheSize = predictedTriangleCount + predictedTriangleCount / 2;
		predictedVertexCount = predictedVertexCount * 4 - 6;
		predictedTriangleCount *= 4;
	}
	
	bool success;
	ht_reserve(cache, EdgeId, int, predictedCacheSize, success, EdgeId_hash);
	dynvec_reserve(vertices, predictedVertexCount, vec3);
	dynvec_reserve(*triangles, predictedTriangleCount, ivec3);
	dynvec_reserve(*tmpTriangles, predictedTriangleCount / 4, ivec3);
	
	for (int i = 0; i < subdivisionCount; i++) {
		subdivideMesh(&vertices, triangles, &cache, tmpTriangles);
		IVec3List *tmp = triangles;
		triangles = tmpTriangles;
		tmpTriangles = tmp;
	}
	
	assert(dynvec_size(vertices) == predictedVertexCount);
	assert(dynvec_size(*triangles) == predictedTriangleCount);
	assert(dynvec_size(*tmpTriangles) == predictedTriangleCount / 4);
	assert(ht_size(cache) == predictedCacheSize);
	
	ht_destroy(cache);
	dynvec_destroy(*tmpTriangles);
	
	IndexedMesh mesh = {
			.vertices = vertices,
			.triangles = *triangles
	};
	return mesh;
}

size_t testIcoSphere(void) {
	IndexedMesh mesh = generateMesh(4);
	size_t result = dynvec_size(mesh.vertices) * dynvec_size(mesh.triangles);
	dynvec_destroy(mesh.triangles);
	dynvec_destroy(mesh.vertices);
	return result;
}

uint64_t getTimestamp() {
#ifndef _MSC_VER
	struct timeval now;
	gettimeofday(&now, NULL);
#else
	static const uint64_t EPOCH = ((uint64_t) 116444736000000000ULL);
	FILETIME nowFileTime;
	GetSystemTimeAsFileTime(&nowFileTime);
	uint64_t time = ((uint64_t) nowFileTime.dwHighDateTime) << 32 | ((uint64_t) nowFileTime.dwLowDateTime);
	time -= EPOCH;
	struct {
		uint64_t tv_sec;
		uint32_t tv_usec;
	} now = { .tv_sec = (time / 10000000L), .tv_usec = time % 10000000 / 10 };
#endif
	return (uint64_t) now.tv_sec * 1000000 + (uint64_t) now.tv_usec;
}

int main() {
	uint64_t start = getTimestamp();
	int count = 10000;
	for (int i = 0; i < count; i++) {
		(void) testIcoSphere();
	}
	uint64_t elapsed = getTimestamp() - start;
	printf("Mesh generation time: %llu us\n", elapsed / count);
	return 0;
}
