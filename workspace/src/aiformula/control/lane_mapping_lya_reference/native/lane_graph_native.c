#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#if defined(_MSC_VER)
#include <intrin.h>
#endif

#ifndef SOURCE_SHA256
#error "setup.py must embed the exact C source SHA256"
#endif

#define MAX_POINTS 4000
#define MAX_QUERIES 4000
#define MAX_CANDIDATES 2048
#define MAX_WORDS ((MAX_POINTS + 63) / 64)
#define GRAPH_CAPSULE "lane_mapping_lya_native.Graph.v1"

#if DBL_MANT_DIG != 53 || DBL_MAX_EXP != 1024
#error "radius_graph requires native IEEE-754 binary64 doubles"
#endif

/* Bounded exact graph: packed rows use little-endian bit order, independently
 * of host alignment/endianness. Python supplies the unchanged distance gate. */
typedef struct {
    int size;
    int words;
    int row_bytes;
    uint64_t tail_mask;
    const unsigned char *packed;
    uint64_t *rows;
    int owned;
} Graph;

static int bounded_int(PyObject *value, const char *name, int maximum, int *result)
{
    long number;
    if (!PyLong_CheckExact(value)) {
        PyErr_Format(PyExc_TypeError, "%s must be a built-in int, not bool", name);
        return 0;
    }
    number = PyLong_AsLong(value);
    if (PyErr_Occurred()) {
        PyErr_Clear();
        PyErr_Format(PyExc_ValueError, "%s must be in [0, %d]", name, maximum);
        return 0;
    }
    if (number < 0 || number > maximum) {
        PyErr_Format(PyExc_ValueError, "%s must be in [0, %d]", name, maximum);
        return 0;
    }
    *result = (int)number;
    return 1;
}

static int exact_bytes(PyObject *value, Py_ssize_t length, const char *name)
{
    if (!PyBytes_CheckExact(value)) {
        PyErr_Format(PyExc_TypeError, "%s must be immutable built-in bytes", name);
        return 0;
    }
    if (PyBytes_GET_SIZE(value) != length) {
        PyErr_Format(PyExc_ValueError, "%s must contain exactly %zd bytes", name, length);
        return 0;
    }
    return 1;
}

static int parse_graph(PyObject *packed, PyObject *size, Graph *graph)
{
    if (!bounded_int(size, "n", MAX_POINTS, &graph->size))
        return 0;
    if (PyCapsule_CheckExact(packed)) {
        Graph *prepared;
        int expected = graph->size;
        if (!PyCapsule_IsValid(packed, GRAPH_CAPSULE)) {
            PyErr_SetString(PyExc_TypeError, "adjacency capsule has a foreign graph type or API version");
            return 0;
        }
        prepared = (Graph *)PyCapsule_GetPointer(packed, GRAPH_CAPSULE);
        if (!prepared)
            return 0;
        if (prepared->size != expected) {
            PyErr_SetString(PyExc_ValueError, "n does not match the prepared graph size");
            return 0;
        }
        *graph = *prepared;
        graph->owned = 0;
        return 1;
    }
    graph->words = (graph->size + 63) / 64;
    graph->row_bytes = (graph->size + 7) / 8;
    graph->tail_mask = (graph->size % 64) ?
        ((UINT64_C(1) << (graph->size % 64)) - 1) : UINT64_MAX;
    if (!exact_bytes(packed, (Py_ssize_t)graph->size * graph->row_bytes, "adjacency"))
        return 0;
    graph->packed = (const unsigned char *)PyBytes_AS_STRING(packed);
    graph->rows = NULL;
    graph->owned = 0;
    return 1;
}

static uint64_t read_le_word(const unsigned char *source, int count)
{
    uint64_t word = 0;
    int index;
    /* Explicit byte assembly is portable and never reads an unaligned uint64. */
    for (index = 0; index < count; ++index)
        word |= (uint64_t)source[index] << (8 * index);
    return word;
}

static void read_row(const Graph *graph, const unsigned char *source, uint64_t *target)
{
    int word;
    for (word = 0; word < graph->words; ++word) {
        int count = graph->row_bytes - 8 * word;
        target[word] = read_le_word(source + 8 * word, count < 8 ? count : 8);
    }
    if (graph->words)
        target[graph->words - 1] &= graph->tail_mask;
}

static void write_row(const Graph *graph, const uint64_t *source, unsigned char *target)
{
    int index;
    for (index = 0; index < graph->row_bytes; ++index)
        target[index] = (unsigned char)(source[index / 8] >> (8 * (index % 8)));
}

static int allocate_graph(Graph *graph)
{
    /* n<=4000 bounds this owned aligned copy to 2,016,000 bytes. */
    size_t length = (size_t)graph->size * graph->words;
    if (graph->rows)
        return 1;
    graph->rows = (uint64_t *)PyMem_Malloc((length ? length : 1) * sizeof(uint64_t));
    if (!graph->rows) {
        PyErr_NoMemory();
        return 0;
    }
    graph->owned = 1;
    return 1;
}

static void free_graph(Graph *graph)
{
    if (graph->owned)
        PyMem_Free(graph->rows);
}

static void decode_graph(Graph *graph)
{
    int row;
    if (!graph->packed)
        return;
    for (row = 0; row < graph->size; ++row)
        read_row(graph, graph->packed + (size_t)row * graph->row_bytes,
                 graph->rows + (size_t)row * graph->words);
}

static void destroy_graph(PyObject *capsule)
{
    Graph *graph = (Graph *)PyCapsule_GetPointer(capsule, GRAPH_CAPSULE);
    if (graph) {
        free_graph(graph);
        PyMem_Free(graph);
    }
}

static PyObject *prepare_graph(PyObject *self, PyObject *args)
{
    PyObject *adjacency, *size, *capsule;
    Graph parsed, *owned;
    (void)self;
    if (!PyArg_ParseTuple(args, "OO:prepare_graph", &adjacency, &size))
        return NULL;
    if (!PyBytes_CheckExact(adjacency)) {
        PyErr_SetString(PyExc_TypeError, "prepare_graph adjacency must be immutable built-in bytes");
        return NULL;
    }
    if (!parse_graph(adjacency, size, &parsed))
        return NULL;
    owned = (Graph *)PyMem_Malloc(sizeof(Graph));
    if (!owned)
        return PyErr_NoMemory();
    *owned = parsed;
    if (!allocate_graph(owned)) {
        PyMem_Free(owned);
        return NULL;
    }
    Py_BEGIN_ALLOW_THREADS
    decode_graph(owned);
    Py_END_ALLOW_THREADS
    /* The capsule owns its complete aligned copy. It neither retains nor
     * borrows Python input storage, and exposes no mutation methods. */
    owned->packed = NULL;
    capsule = PyCapsule_New(owned, GRAPH_CAPSULE, destroy_graph);
    if (!capsule) {
        free_graph(owned);
        PyMem_Free(owned);
    }
    return capsule;
}

static int lowest_word_bit(uint64_t word)
{
#if defined(_MSC_VER) && defined(_M_X64)
    unsigned long index;
    _BitScanForward64(&index, word);
    return (int)index;
#elif defined(__GNUC__) || defined(__clang__)
    return (int)__builtin_ctzll(word);
#else
    int index = 0;
    while (!(word & UINT64_C(1))) {
        ++index;
        word >>= 1;
    }
    return index;
#endif
}

static int lowest_bit(const uint64_t *bits, int words)
{
    int word;
    for (word = 0; word < words; ++word)
        if (bits[word])
            return 64 * word + lowest_word_bit(bits[word]);
    return -1;
}

static void flood_subset(const Graph *graph, const uint64_t *allowed, int seed,
                         uint64_t *reached)
{
    uint64_t frontier[MAX_WORDS];
    int node;
    memset(reached, 0, (size_t)graph->words * sizeof(uint64_t));
    if (!(allowed[seed / 64] & (UINT64_C(1) << (seed % 64))))
        return;
    memset(frontier, 0, (size_t)graph->words * sizeof(uint64_t));
    reached[seed / 64] = frontier[seed / 64] = UINT64_C(1) << (seed % 64);
    while (memcmp(reached, allowed, (size_t)graph->words * sizeof(uint64_t)) != 0 &&
           (node = lowest_bit(frontier, graph->words)) >= 0) {
        int word;
        const uint64_t *neighbors = graph->rows + (size_t)node * graph->words;
        frontier[node / 64] &= ~(UINT64_C(1) << (node % 64));
        for (word = 0; word < graph->words; ++word) {
            uint64_t added = neighbors[word] & allowed[word] & ~reached[word];
            reached[word] |= added;
            frontier[word] |= added;
        }
    }
}

static int *copy_seeds(PyObject *sequence, int size, Py_ssize_t *length)
{
    int *seeds;
    Py_ssize_t index;
    if (!PySequence_Check(sequence) || PyUnicode_Check(sequence) || PyBytes_Check(sequence)) {
        PyErr_SetString(PyExc_TypeError, "seeds must be a finite integer sequence");
        return NULL;
    }
    *length = PySequence_Size(sequence);
    if (*length < 0)
        return NULL;
    if (*length > MAX_QUERIES) {
        PyErr_SetString(PyExc_ValueError, "seeds must contain at most 4000 queries");
        return NULL;
    }
    seeds = (int *)PyMem_Malloc((size_t)(*length ? *length : 1) * sizeof(int));
    if (!seeds) {
        PyErr_NoMemory();
        return NULL;
    }
    for (index = 0; index < *length; ++index) {
        PyObject *item = PySequence_GetItem(sequence, index);
        int valid;
        if (!item) {
            PyMem_Free(seeds);
            return NULL;
        }
        valid = bounded_int(item, "seed", size - 1, &seeds[index]);
        Py_DECREF(item);
        if (!valid) {
            PyMem_Free(seeds);
            return NULL;
        }
    }
    return seeds;
}

static PyObject *connected_subsets(PyObject *self, PyObject *args)
{
    PyObject *adjacency, *size, *allowed_object, *sequence, *output;
    const unsigned char *allowed;
    unsigned char *result;
    Graph graph;
    int *seeds;
    Py_ssize_t queries, row;
    (void)self;
    if (!PyArg_ParseTuple(args, "OOOO:connected_subsets", &adjacency, &size, &allowed_object, &sequence))
        return NULL;
    if (!parse_graph(adjacency, size, &graph))
        return NULL;
    seeds = copy_seeds(sequence, graph.size, &queries);
    if (!seeds)
        return NULL;
    if (!exact_bytes(allowed_object, queries * graph.row_bytes, "allowed")) {
        PyMem_Free(seeds);
        return NULL;
    }
    if (!allocate_graph(&graph)) {
        PyMem_Free(seeds);
        return NULL;
    }
    output = PyBytes_FromStringAndSize(NULL, queries * graph.row_bytes);
    if (!output) {
        free_graph(&graph);
        PyMem_Free(seeds);
        return NULL;
    }
    allowed = (const unsigned char *)PyBytes_AS_STRING(allowed_object);
    result = (unsigned char *)PyBytes_AS_STRING(output);
    /* Borrowed inputs are immutable bytes kept alive by args. Seeds are copied,
     * and the new output has not escaped. No Python API executes without GIL. */
    Py_BEGIN_ALLOW_THREADS
    decode_graph(&graph);
    for (row = 0; row < queries; ++row) {
        uint64_t mask[MAX_WORDS], reached[MAX_WORDS];
        read_row(&graph, allowed + (size_t)row * graph.row_bytes, mask);
        flood_subset(&graph, mask, seeds[row], reached);
        write_row(&graph, reached, result + (size_t)row * graph.row_bytes);
    }
    Py_END_ALLOW_THREADS
    free_graph(&graph);
    PyMem_Free(seeds);
    return output;
}

static void build_forest(const Graph *graph, int *labels, int *parents)
{
    uint64_t unseen[MAX_WORDS], reached[MAX_WORDS], frontier[MAX_WORDS];
    int seed, index;
    for (index = 0; index < graph->size; ++index) {
        labels[index] = index;
        parents[index] = -1;
    }
    for (index = 0; index < graph->words; ++index)
        unseen[index] = UINT64_MAX;
    if (graph->words)
        unseen[graph->words - 1] &= graph->tail_mask;
    while ((seed = lowest_bit(unseen, graph->words)) >= 0) {
        int node, word;
        memset(reached, 0, (size_t)graph->words * sizeof(uint64_t));
        memset(frontier, 0, (size_t)graph->words * sizeof(uint64_t));
        reached[seed / 64] = frontier[seed / 64] = UINT64_C(1) << (seed % 64);
        while ((node = lowest_bit(frontier, graph->words)) >= 0) {
            const uint64_t *neighbors = graph->rows + (size_t)node * graph->words;
            frontier[node / 64] &= ~(UINT64_C(1) << (node % 64));
            for (word = 0; word < graph->words; ++word) {
                uint64_t added = neighbors[word] & ~reached[word];
                uint64_t children = added;
                /* Assign every first-discovered child before queueing it,
                 * matching R5's lowest-bit frontier and parent assignment. */
                while (children) {
                    int bit = lowest_word_bit(children);
                    parents[word * 64 + bit] = node;
                    children &= children - 1;
                }
                reached[word] |= added;
                frontier[word] |= added;
            }
        }
        for (word = 0; word < graph->words; ++word) {
            uint64_t chosen = reached[word];
            while (chosen) {
                int bit = lowest_word_bit(chosen);
                labels[word * 64 + bit] = seed;
                chosen &= chosen - 1;
            }
            unseen[word] &= ~reached[word];
        }
    }
}

static PyObject *graph_forest(PyObject *self, PyObject *args)
{
    PyObject *adjacency, *size, *labels_list, *parents_list, *output;
    Graph graph;
    int *labels, *parents, index;
    (void)self;
    if (!PyArg_ParseTuple(args, "OO:graph_forest", &adjacency, &size))
        return NULL;
    if (!parse_graph(adjacency, size, &graph) || !allocate_graph(&graph))
        return NULL;
    labels = (int *)PyMem_Malloc((size_t)(graph.size ? graph.size : 1) * sizeof(int));
    parents = (int *)PyMem_Malloc((size_t)(graph.size ? graph.size : 1) * sizeof(int));
    if (!labels || !parents) {
        free_graph(&graph);
        PyMem_Free(labels);
        PyMem_Free(parents);
        return PyErr_NoMemory();
    }
    Py_BEGIN_ALLOW_THREADS
    decode_graph(&graph);
    build_forest(&graph, labels, parents);
    Py_END_ALLOW_THREADS
    free_graph(&graph);
    labels_list = PyList_New(graph.size);
    parents_list = PyList_New(graph.size);
    if (!labels_list || !parents_list) {
        Py_XDECREF(labels_list);
        Py_XDECREF(parents_list);
        PyMem_Free(labels);
        PyMem_Free(parents);
        return NULL;
    }
    for (index = 0; index < graph.size; ++index) {
        PyObject *label = PyLong_FromLong(labels[index]);
        PyObject *parent = PyLong_FromLong(parents[index]);
        if (!label || !parent) {
            Py_XDECREF(label);
            Py_XDECREF(parent);
            Py_DECREF(labels_list);
            Py_DECREF(parents_list);
            PyMem_Free(labels);
            PyMem_Free(parents);
            return NULL;
        }
        PyList_SET_ITEM(labels_list, index, label);
        PyList_SET_ITEM(parents_list, index, parent);
    }
    PyMem_Free(labels);
    PyMem_Free(parents);
    output = PyTuple_Pack(2, labels_list, parents_list);
    Py_DECREF(labels_list);
    Py_DECREF(parents_list);
    return output;
}

static int adjacent_key(int64_t first, int64_t second)
{
    if (first == second)
        return 1;
    /* The smaller signed value cannot be INT64_MAX. This avoids abs(a-b),
     * whose subtraction can overflow for perfectly valid int64 endpoints. */
    return first < second ? first + INT64_C(1) == second : second + INT64_C(1) == first;
}

static PyObject *radius_graph(PyObject *self, PyObject *args)
{
    PyObject *point_bytes, *key_bytes, *size, *threshold_object, *output;
    double *points, radius_squared;
    int64_t *keys;
    unsigned char *packed;
    int count, row_bytes, coordinate, left;
    size_t input_bytes;
    (void)self;
    if (!PyArg_ParseTuple(args, "OOOO:radius_graph", &point_bytes, &key_bytes, &size, &threshold_object))
        return NULL;
    if (!bounded_int(size, "n", MAX_POINTS, &count))
        return NULL;
    input_bytes = (size_t)count * 2 * sizeof(double);
    if (!exact_bytes(point_bytes, (Py_ssize_t)input_bytes, "points") ||
        !exact_bytes(key_bytes, (Py_ssize_t)count * 2 * sizeof(int64_t), "keys"))
        return NULL;
    if (!PyFloat_CheckExact(threshold_object)) {
        PyErr_SetString(PyExc_TypeError, "radius_squared must be a built-in float");
        return NULL;
    }
    radius_squared = PyFloat_AS_DOUBLE(threshold_object);
    if (!(radius_squared >= 0.0)) {
        PyErr_SetString(PyExc_ValueError, "radius_squared must be nonnegative and not NaN");
        return NULL;
    }
    points = (double *)PyMem_Malloc(input_bytes ? input_bytes : sizeof(double));
    keys = (int64_t *)PyMem_Malloc(input_bytes ? input_bytes : sizeof(int64_t));
    if (!points || !keys) {
        PyMem_Free(points);
        PyMem_Free(keys);
        return PyErr_NoMemory();
    }
    /* Input has native f64/int64 byte order. memcpy permits unaligned byte
     * storage; the owned arrays remain valid throughout the GIL-free loop. */
    memcpy(points, PyBytes_AS_STRING(point_bytes), input_bytes);
    memcpy(keys, PyBytes_AS_STRING(key_bytes), input_bytes);
    for (coordinate = 0; coordinate < 2 * count; ++coordinate) {
        if (!(points[coordinate] >= -1e7 && points[coordinate] <= 1e7)) {
            PyMem_Free(points);
            PyMem_Free(keys);
            PyErr_Format(PyExc_ValueError, "points[%d] must be finite and inside [-1e7, 1e7]", coordinate);
            return NULL;
        }
    }
    row_bytes = (count + 7) / 8;
    output = PyBytes_FromStringAndSize(NULL, (Py_ssize_t)count * row_bytes);
    if (!output) {
        PyMem_Free(points);
        PyMem_Free(keys);
        return NULL;
    }
    packed = (unsigned char *)PyBytes_AS_STRING(output);
    memset(packed, 0, (size_t)count * row_bytes);
    Py_BEGIN_ALLOW_THREADS
    /* Deliberately preserve separate binary64 subtraction, square and sum.
     * Volatile stores forbid contraction into FMA or extended intermediates.
     * The caller supplies the original Python radius ** 2 threshold, preserving
     * its rounding/overflow before this function. No pow or radius arithmetic
     * is performed here. Zero (underflow) and positive infinity are valid
     * explicit thresholds; Python decides whether a source radius is valid. */
    for (left = 0; left < count; ++left) {
        int right;
        for (right = left; right < count; ++right) {
            volatile double dx, dy, dx_squared, dy_squared, squared;
            if (!adjacent_key(keys[2 * left], keys[2 * right]) ||
                !adjacent_key(keys[2 * left + 1], keys[2 * right + 1]))
                continue;
            dx = points[2 * left] - points[2 * right];
            dy = points[2 * left + 1] - points[2 * right + 1];
            dx_squared = dx * dx;
            dy_squared = dy * dy;
            squared = dx_squared + dy_squared;
            if (squared <= radius_squared) {
                packed[(size_t)left * row_bytes + right / 8] |= (unsigned char)(1u << (right % 8));
                packed[(size_t)right * row_bytes + left / 8] |= (unsigned char)(1u << (left % 8));
            }
        }
    }
    Py_END_ALLOW_THREADS
    PyMem_Free(points);
    PyMem_Free(keys);
    return output;
}

/* One bounded allocation per call, rather than Q x n Python masks. All ids
 * are copied from native-int64 bytes with memcpy before entering the kernel. */
typedef struct {
    int candidates[MAX_CANDIDATES];
    int targets[MAX_QUERIES];
    int parents[MAX_POINTS];
    int positions[MAX_POINTS];
    int parent_positions[MAX_CANDIDATES];
    int roots[MAX_POINTS];
    unsigned char ranks[MAX_POINTS];
} SupportWork;

static int index_bytes_length(PyObject *value, const char *name, int maximum, int *count)
{
    Py_ssize_t length;
    if (!PyBytes_CheckExact(value)) {
        PyErr_Format(PyExc_TypeError, "%s must be immutable built-in bytes", name);
        return 0;
    }
    length = PyBytes_GET_SIZE(value);
    if (length % sizeof(int64_t) || length / sizeof(int64_t) > maximum) {
        PyErr_Format(PyExc_ValueError, "%s must contain at most %d native int64 values", name, maximum);
        return 0;
    }
    *count = (int)(length / sizeof(int64_t));
    return 1;
}

static int copy_support_ids(PyObject *value, int count, int size, const char *name, int *output)
{
    int index;
    const char *bytes = PyBytes_AS_STRING(value);
    for (index = 0; index < count; ++index) {
        int64_t vertex;
        memcpy(&vertex, bytes + (size_t)index * sizeof(int64_t), sizeof(vertex));
        if (vertex < 0 || vertex >= size) {
            PyErr_Format(PyExc_ValueError, "%s[%d] must be in [0, n)", name, index);
            return 0;
        }
        output[index] = (int)vertex;
    }
    return 1;
}

static int support_root(int *roots, int vertex)
{
    while (roots[vertex] != vertex) {
        roots[vertex] = roots[roots[vertex]];
        vertex = roots[vertex];
    }
    return vertex;
}

static int validate_support_forest(const Graph *graph, PyObject *parent_bytes, SupportWork *work)
{
    int vertex;
    const char *bytes = PyBytes_AS_STRING(parent_bytes);
    for (vertex = 0; vertex < graph->size; ++vertex) {
        int64_t parent;
        memcpy(&parent, bytes + (size_t)vertex * sizeof(int64_t), sizeof(parent));
        if (parent < -1 || parent >= graph->size) {
            PyErr_Format(PyExc_ValueError, "parents[%d] must be -1 or in [0, n)", vertex);
            return 0;
        }
        work->parents[vertex] = (int)parent;
        work->roots[vertex] = vertex;
        work->ranks[vertex] = 0;
        work->positions[vertex] = -1;
    }
    for (vertex = 0; vertex < graph->size; ++vertex) {
        int parent = work->parents[vertex], left, right;
        const uint64_t *neighbors, *reverse;
        if (parent < 0)
            continue;
        neighbors = graph->rows + (size_t)vertex * graph->words;
        reverse = graph->rows + (size_t)parent * graph->words;
        /* The certificate must work from any allowed seed. Requiring both
         * directed edges makes each accepted forest edge genuinely undirected,
         * even if the caller supplied a graph with unrelated directed edges. */
        if (!(neighbors[parent / 64] & (UINT64_C(1) << (parent % 64))) ||
            !(reverse[vertex / 64] & (UINT64_C(1) << (vertex % 64)))) {
            PyErr_Format(PyExc_ValueError, "parents[%d] is not a bidirectional graph edge", vertex);
            return 0;
        }
        left = support_root(work->roots, vertex);
        right = support_root(work->roots, parent);
        if (left == right) {
            PyErr_Format(PyExc_ValueError, "parents contain a cycle at vertex %d", vertex);
            return 0;
        }
        if (work->ranks[left] < work->ranks[right]) {
            work->roots[left] = right;
        } else {
            work->roots[right] = left;
            if (work->ranks[left] == work->ranks[right])
                ++work->ranks[left];
        }
    }
    return 1;
}

static void support_rows(const Graph *graph, const SupportWork *work, int columns,
                         int queries, const unsigned char *allowed,
                         unsigned char *support, unsigned char *counts)
{
    int row;
    for (row = 0; row < queries; ++row) {
        const unsigned char *input = allowed + (size_t)row * columns;
        unsigned char *output = support + (size_t)row * columns;
        int seed = work->targets[row], seed_position = work->positions[seed];
        int column, edge_count = 0;
        int64_t count = 0;
        uint64_t mask[MAX_WORDS], reached[MAX_WORDS];
        memset(output, 0, (size_t)columns);
        /* A connected set is never evidence for a seed outside that set. */
        if (seed_position < 0 || !input[seed_position]) {
            memcpy(counts + (size_t)row * sizeof(count), &count, sizeof(count));
            continue;
        }
        for (column = 0; column < columns; ++column) {
            int parent = work->parent_positions[column];
            if (!input[column])
                continue;
            ++count;
            if (parent >= 0 && input[parent])
                ++edge_count;
        }
        /* A subset of an acyclic bidirectional forest is connected iff it
         * retains vertices-1 edges. No global-component shortcut is needed. */
        if (edge_count == count - 1) {
            memcpy(output, input, (size_t)columns);
        } else {
            memset(mask, 0, (size_t)graph->words * sizeof(uint64_t));
            for (column = 0; column < columns; ++column) {
                int vertex = work->candidates[column];
                if (input[column])
                    mask[vertex / 64] |= UINT64_C(1) << (vertex % 64);
            }
            flood_subset(graph, mask, seed, reached);
            count = 0;
            for (column = 0; column < columns; ++column) {
                int vertex = work->candidates[column];
                output[column] = (unsigned char)((reached[vertex / 64] >> (vertex % 64)) & 1);
                count += output[column];
            }
        }
        /* Counts deliberately use native int64 byte order, matching Python
         * np.frombuffer(count_bytes, dtype=np.int64), without alignment casts. */
        memcpy(counts + (size_t)row * sizeof(count), &count, sizeof(count));
    }
}

static PyObject *local_support(PyObject *self, PyObject *args)
{
    PyObject *capsule, *size, *candidate_bytes, *target_bytes, *allowed_bytes, *parent_bytes;
    PyObject *support = NULL, *counts = NULL, *output = NULL;
    Graph graph;
    SupportWork *work = NULL;
    int columns, queries, column;
    Py_ssize_t offset, allowed_size, invalid_offset = -1;
    const unsigned char *allowed;
    unsigned char *support_data, *count_data;
    (void)self;
    if (!PyArg_ParseTuple(args, "OOOOOO:local_support", &capsule, &size,
                         &candidate_bytes, &target_bytes, &allowed_bytes, &parent_bytes))
        return NULL;
    if (!PyCapsule_CheckExact(capsule)) {
        PyErr_SetString(PyExc_TypeError, "local_support requires a prepared graph capsule");
        return NULL;
    }
    if (!parse_graph(capsule, size, &graph) ||
        !index_bytes_length(candidate_bytes, "candidates", MAX_CANDIDATES, &columns) ||
        !index_bytes_length(target_bytes, "targets", MAX_QUERIES, &queries))
        return NULL;
    allowed_size = (Py_ssize_t)queries * columns;
    if (!exact_bytes(allowed_bytes, allowed_size, "allowed") ||
        !exact_bytes(parent_bytes, (Py_ssize_t)graph.size * sizeof(int64_t), "parents"))
        return NULL;
    work = (SupportWork *)PyMem_Malloc(sizeof(SupportWork));
    if (!work)
        return PyErr_NoMemory();
    if (!copy_support_ids(candidate_bytes, columns, graph.size, "candidates", work->candidates) ||
        !copy_support_ids(target_bytes, queries, graph.size, "targets", work->targets) ||
        !validate_support_forest(&graph, parent_bytes, work))
        goto cleanup;
    for (column = 0; column < columns; ++column) {
        int vertex = work->candidates[column];
        if (work->positions[vertex] >= 0) {
            PyErr_Format(PyExc_ValueError, "candidates[%d] duplicates an earlier vertex", column);
            goto cleanup;
        }
        work->positions[vertex] = column;
    }
    for (column = 0; column < columns; ++column) {
        int parent = work->parents[work->candidates[column]];
        work->parent_positions[column] = parent < 0 ? -1 : work->positions[parent];
    }
    support = PyBytes_FromStringAndSize(NULL, allowed_size);
    counts = PyBytes_FromStringAndSize(NULL, (Py_ssize_t)queries * sizeof(int64_t));
    if (!support || !counts)
        goto cleanup;
    allowed = (const unsigned char *)PyBytes_AS_STRING(allowed_bytes);
    support_data = (unsigned char *)PyBytes_AS_STRING(support);
    count_data = (unsigned char *)PyBytes_AS_STRING(counts);
    /* args retain the immutable graph/bytes; ids and forest are owned copies.
     * Validate every allowed byte, including rows whose seed is absent. Reject
     * before computing any result. No Python API runs without the GIL. */
    Py_BEGIN_ALLOW_THREADS
    for (offset = 0; offset < allowed_size; ++offset) {
        if (allowed[offset] > 1) {
            invalid_offset = offset;
            break;
        }
    }
    if (invalid_offset < 0)
        support_rows(&graph, work, columns, queries, allowed, support_data, count_data);
    Py_END_ALLOW_THREADS
    if (invalid_offset >= 0) {
        PyErr_Format(PyExc_ValueError, "allowed[%zd] must be 0 or 1", invalid_offset);
        goto cleanup;
    }
    output = PyTuple_Pack(2, support, counts);
cleanup:
    Py_XDECREF(support);
    Py_XDECREF(counts);
    PyMem_Free(work);
    return output;
}

/* Exact cross-section weights.
 *
 * Existing RC4 helpers: Graph, parse_graph, exact_bytes, index_bytes_length,
 * copy_support_ids, flood_subset, MAX_WORDS and MAX_QUERIES.
 * Register cross_section_weights with METH_VARARGS and expose
 * CROSS_SECTION_WEIGHTS_API_VERSION=1. No float threshold is recomputed here.
 *
 * Numeric scope: native IEEE f64, finite cloud coordinates within +/-1e7,
 * arbitrary finite normals (not assumed normalized), nonnegative thresholds.
 * Intermediate overflow follows IEEE Inf/NaN comparison semantics. This C API
 * does not emit NumPy's seterr warnings/FloatingPointError; callers needing
 * those side effects must keep the original NumPy path on overflow-risk input.
 */

static int weights_threshold(PyObject *value, const char *name, double *result)
{
    if (!PyFloat_CheckExact(value)) {
        PyErr_Format(PyExc_TypeError, "%s must be a built-in float", name);
        return 0;
    }
    *result = PyFloat_AS_DOUBLE(value);
    if (!(*result >= 0.0)) {
        PyErr_Format(PyExc_ValueError, "%s must be nonnegative and not NaN", name);
        return 0;
    }
    return 1;
}

static void cross_section_kernel(const Graph *graph, const double *cloud,
                                 const double *normals, const int *ids, int queries,
                                 double half_slab, double radius_squared,
                                 unsigned char *output)
{
    int row;
    for (row = 0; row < queries; ++row) {
        uint64_t allowed[MAX_WORDS], reached[MAX_WORDS];
        int vertex, item, count = 0, seed = ids[row];
        double weight;
        memset(allowed, 0, (size_t)graph->words * sizeof(uint64_t));
        for (vertex = 0; vertex < graph->size; ++vertex) {
            /* Match NumPy's original expression's independent binary64 ufunc
             * results. Each product has its own volatile store, so the compiler
             * cannot fuse along into FMA or retain extended intermediates. */
            volatile double dx, dy, along_x, along_y, along;
            volatile double squared_x, squared_y, squared;
            dx = cloud[2 * vertex] - cloud[2 * seed];
            dy = cloud[2 * vertex + 1] - cloud[2 * seed + 1];
            along_x = dx * -normals[2 * row + 1];
            along_y = dy * normals[2 * row];
            along = along_x + along_y;
            squared_x = dx * dx;
            squared_y = dy * dy;
            squared = squared_x + squared_y;
            if (fabs(along) <= half_slab && squared <= radius_squared)
                allowed[vertex / 64] |= UINT64_C(1) << (vertex % 64);
        }
        flood_subset(graph, allowed, seed, reached);
        /* Every ID is counted, not just distinct support vertices. Two reliable
         * voxels mapped to one full-support seed retain multiplicity exactly. */
        for (item = 0; item < queries; ++item) {
            int vertex_id = ids[item];
            count += (int)((reached[vertex_id / 64] >> (vertex_id % 64)) & 1);
        }
        /* Valid finite inputs and nonnegative thresholds always admit the seed:
         * dx=dy=0, and that same ID occurs in the reliable list. count>=1. */
        weight = 1.0 / (double)count;
        memcpy(output + (size_t)row * sizeof(double), &weight, sizeof(weight));
    }
}

static PyObject *cross_section_weights(PyObject *self, PyObject *args)
{
    PyObject *capsule, *size, *cloud_bytes, *normal_bytes, *id_bytes;
    PyObject *half_slab_object, *radius_squared_object, *output;
    Graph graph;
    double half_slab, radius_squared, *cloud, *normals;
    int queries, coordinate, *ids;
    unsigned char *storage, *weights;
    size_t cloud_size, normal_size, id_size, total_size;
    (void)self;
    if (!PyArg_ParseTuple(args, "OOOOOOO:cross_section_weights", &capsule, &size,
                         &cloud_bytes, &normal_bytes, &id_bytes,
                         &half_slab_object, &radius_squared_object))
        return NULL;
    if (!PyCapsule_CheckExact(capsule)) {
        PyErr_SetString(PyExc_TypeError, "cross_section_weights requires a prepared graph capsule");
        return NULL;
    }
    if (!parse_graph(capsule, size, &graph) ||
        !index_bytes_length(id_bytes, "support_ids", MAX_QUERIES, &queries))
        return NULL;
    cloud_size = (size_t)graph.size * 2 * sizeof(double);
    normal_size = (size_t)queries * 2 * sizeof(double);
    id_size = (size_t)queries * sizeof(int);
    if (!exact_bytes(cloud_bytes, (Py_ssize_t)cloud_size, "cloud") ||
        !exact_bytes(normal_bytes, (Py_ssize_t)normal_size, "normals") ||
        !weights_threshold(half_slab_object, "half_slab", &half_slab) ||
        !weights_threshold(radius_squared_object, "radius_squared", &radius_squared))
        return NULL;
    /* At most 144 KB. Double blocks start aligned; the int block follows whole
     * doubles. memcpy accepts unaligned Python bytes and native endian input. */
    total_size = cloud_size + normal_size + id_size;
    storage = (unsigned char *)PyMem_Malloc(total_size ? total_size : sizeof(double));
    if (!storage)
        return PyErr_NoMemory();
    cloud = (double *)storage;
    normals = (double *)(storage + cloud_size);
    ids = (int *)(storage + cloud_size + normal_size);
    memcpy(cloud, PyBytes_AS_STRING(cloud_bytes), cloud_size);
    memcpy(normals, PyBytes_AS_STRING(normal_bytes), normal_size);
    if (!copy_support_ids(id_bytes, queries, graph.size, "support_ids", ids)) {
        PyMem_Free(storage);
        return NULL;
    }
    for (coordinate = 0; coordinate < 2 * graph.size; ++coordinate) {
        if (!(cloud[coordinate] >= -1e7 && cloud[coordinate] <= 1e7)) {
            PyMem_Free(storage);
            PyErr_Format(PyExc_ValueError, "cloud[%d] must be finite and inside [-1e7, 1e7]", coordinate);
            return NULL;
        }
    }
    for (coordinate = 0; coordinate < 2 * queries; ++coordinate) {
        if (!Py_IS_FINITE(normals[coordinate])) {
            PyMem_Free(storage);
            PyErr_Format(PyExc_ValueError, "normals[%d] must be finite", coordinate);
            return NULL;
        }
    }
    output = PyBytes_FromStringAndSize(NULL, (Py_ssize_t)queries * sizeof(double));
    if (!output) {
        PyMem_Free(storage);
        return NULL;
    }
    weights = (unsigned char *)PyBytes_AS_STRING(output);
    /* args keep the immutable prepared graph alive. Work arrays are owned,
     * output has not escaped, and no Python API executes without the GIL. */
    Py_BEGIN_ALLOW_THREADS
    cross_section_kernel(&graph, cloud, normals, ids, queries,
                         half_slab, radius_squared, weights);
    Py_END_ALLOW_THREADS
    PyMem_Free(storage);
    return output;
}

/* Exact grouped distance reduction.
 * No dot product, geometry generation, sorting, truncation or timing override.
 * Python supplies the ORIGINAL per-group NumPy @ compatibility result.
 * Build with strict IEEE semantics: no fast-math, no contraction/FMA.
 * Inputs are native-endian immutable bytes. No NumPy C API is required.
 * Bounds: 4000 queries, 4000 anchors, 4000 groups, 2048 candidates/group.
 * Candidate IDs <=8,192,000 int64 =65,536,000 bytes (not 32 MB).
 * Allowed masks <=8,192,000 bytes; no pair-distance matrix is allocated.
 */
#define NN_MAX_POINTS 4000
#define NN_MAX_CANDIDATES 2048
#define NN_MAX_PAIRS ((int64_t)NN_MAX_POINTS * NN_MAX_CANDIDATES)

static int64_t nn_i64(const char *data, Py_ssize_t index) {
    int64_t value;
    memcpy(&value, data + index * 8, 8);
    return value;
}

static double nn_f64(const char *data, Py_ssize_t index) {
    double value;
    memcpy(&value, data + index * 8, 8);
    return value;
}

static int nn_bytes(PyObject *object, Py_ssize_t width, const char **data,
                    Py_ssize_t *count, const char *name) {
    Py_ssize_t length;
    if (!PyBytes_CheckExact(object)) {
        PyErr_Format(PyExc_TypeError, "%s must be immutable bytes", name);
        return 0;
    }
    length = PyBytes_GET_SIZE(object);
    if (length % width) {
        PyErr_Format(PyExc_ValueError, "%s has an invalid byte length", name);
        return 0;
    }
    *data = PyBytes_AS_STRING(object);
    *count = length / width;
    return 1;
}

static PyObject *nearest_grouped(PyObject *self, PyObject *args) {
    PyObject *q_obj, *a_obj, *qo_obj, *qi_obj, *co_obj, *ci_obj, *mask_obj, *limit_obj;
    PyObject *output = NULL;
    const char *q, *a, *qo, *qi, *co, *ci, *mask = NULL;
    Py_ssize_t nq, na, nqo, nqi, nco, nci, nmask = 0, groups, g, k;
    unsigned char *seen = NULL;
    char *out;
    int64_t pair_count = 0, previous_q = 0, previous_c = 0;
    double limit;
    (void)self;
    if (!PyArg_ParseTuple(args, "OOOOOOOO:nearest_grouped", &q_obj, &a_obj,
                          &qo_obj, &qi_obj, &co_obj, &ci_obj, &mask_obj, &limit_obj))
        return NULL;
    if (sizeof(double) != 8 || sizeof(int64_t) != 8 || FLT_RADIX != 2 ||
            DBL_MANT_DIG != 53 || DBL_MAX_EXP != 1024) {
        PyErr_SetString(PyExc_RuntimeError, "native IEEE binary64/int64 required");
        return NULL;
    }
    if (!PyFloat_CheckExact(limit_obj)) {
        PyErr_SetString(PyExc_TypeError, "squared limit must be a built-in float");
        return NULL;
    }
    limit = PyFloat_AS_DOUBLE(limit_obj);
    if (Py_IS_NAN(limit) || limit < 0.0) {
        PyErr_SetString(PyExc_ValueError, "squared limit must be nonnegative and not NaN");
        return NULL;
    }
    if (!nn_bytes(q_obj, 16, &q, &nq, "query_xy") ||
            !nn_bytes(a_obj, 16, &a, &na, "anchor_xy") ||
            !nn_bytes(qo_obj, 8, &qo, &nqo, "query_offsets") ||
            !nn_bytes(qi_obj, 8, &qi, &nqi, "query_ids") ||
            !nn_bytes(co_obj, 8, &co, &nco, "candidate_offsets") ||
            !nn_bytes(ci_obj, 8, &ci, &nci, "candidate_ids"))
        return NULL;
    if (mask_obj != Py_None && !nn_bytes(mask_obj, 1, &mask, &nmask, "allowed"))
        return NULL;
    if (nq > NN_MAX_POINTS || na > NN_MAX_POINTS || nqo < 1 || nco != nqo ||
            nqo > NN_MAX_POINTS + 1 || nqi != nq || nci > NN_MAX_PAIRS) {
        PyErr_SetString(PyExc_ValueError, "bounded nearest dimensions violated");
        return NULL;
    }
    groups = nqo - 1;
    if (groups > nq || nn_i64(qo, 0) != 0 || nn_i64(co, 0) != 0 ||
            nn_i64(qo, groups) != nqi || nn_i64(co, groups) != nci) {
        PyErr_SetString(PyExc_ValueError, "CSR offsets or complete query coverage invalid");
        return NULL;
    }
    for (g = 0; g < groups; ++g) {
        int64_t end_q = nn_i64(qo, g + 1), end_c = nn_i64(co, g + 1);
        int64_t width, rows;
        if (end_q < previous_q || end_q > nqi || end_c < previous_c || end_c > nci) {
            PyErr_SetString(PyExc_ValueError, "CSR offsets must be bounded and monotonic");
            return NULL;
        }
        width = end_c - previous_c;
        rows = end_q - previous_q;
        if (width > NN_MAX_CANDIDATES) {
            PyErr_SetString(PyExc_ValueError, "candidate group exceeds 2048; never truncate in C");
            return NULL;
        }
        pair_count += rows * width; /* bounded by 4000 * 2048 */
        previous_q = end_q;
        previous_c = end_c;
    }
    if (pair_count > NN_MAX_PAIRS || (mask && nmask != pair_count)) {
        PyErr_SetString(PyExc_ValueError, "allowed byte length does not match grouped pairs");
        return NULL;
    }
    for (k = 0; k < nq * 2; ++k) {
        if (!Py_IS_FINITE(nn_f64(q, k))) {
            PyErr_SetString(PyExc_ValueError, "query coordinates must be finite");
            return NULL;
        }
    }
    for (k = 0; k < na * 2; ++k) {
        if (!Py_IS_FINITE(nn_f64(a, k))) {
            PyErr_SetString(PyExc_ValueError, "anchor coordinates must be finite");
            return NULL;
        }
    }
    seen = PyMem_Calloc(nq ? nq : 1, 1);
    if (!seen) return PyErr_NoMemory();
    for (k = 0; k < nqi; ++k) {
        int64_t id = nn_i64(qi, k);
        if (id < 0 || id >= nq || seen[id]) {
            PyErr_SetString(PyExc_ValueError, "query IDs must cover every query exactly once");
            goto fail;
        }
        seen[id] = 1;
    }
    for (k = 0; k < nci; ++k) {
        int64_t id = nn_i64(ci, k);
        if (id < 0 || id >= na) {
            PyErr_SetString(PyExc_ValueError, "candidate ID outside anchor");
            goto fail;
        }
    }
    for (k = 0; mask && k < nmask; ++k) {
        if ((unsigned char)mask[k] > 1) {
            PyErr_SetString(PyExc_ValueError, "allowed values must be 0 or 1");
            goto fail;
        }
    }
    PyMem_Free(seen);
    seen = NULL;
    output = PyBytes_FromStringAndSize(NULL, nq * 8);
    if (!output) return NULL;
    out = PyBytes_AS_STRING(output);
    /* Objects are immutable and retained by args. No Python API/allocation is
       called while the GIL is released. The new output is not yet exposed. */
    Py_BEGIN_ALLOW_THREADS
    {
        Py_ssize_t pair_offset = 0;
        for (g = 0; g < groups; ++g) {
            Py_ssize_t q_start = (Py_ssize_t)nn_i64(qo, g);
            Py_ssize_t q_end = (Py_ssize_t)nn_i64(qo, g + 1);
            Py_ssize_t c_start = (Py_ssize_t)nn_i64(co, g);
            Py_ssize_t c_end = (Py_ssize_t)nn_i64(co, g + 1);
            Py_ssize_t width = c_end - c_start, row;
            for (row = q_start; row < q_end; ++row) {
                int64_t query_id = nn_i64(qi, row), best_id = -1;
                double best = HUGE_VAL;
                double x = nn_f64(q, (Py_ssize_t)query_id * 2);
                double y = nn_f64(q, (Py_ssize_t)query_id * 2 + 1);
                Py_ssize_t column;
                for (column = c_start; column < c_end; ++column) {
                    int64_t candidate_id = nn_i64(ci, column);
                    double distance = HUGE_VAL;
                    Py_ssize_t at = pair_offset + (row - q_start) * width + column - c_start;
                    if (!mask || mask[at]) {
                        volatile double dx = x - nn_f64(a, (Py_ssize_t)candidate_id * 2);
                        volatile double dy = y - nn_f64(a, (Py_ssize_t)candidate_id * 2 + 1);
                        volatile double xx = dx * dx;
                        volatile double yy = dy * dy;
                        volatile double sum = xx + yy;
                        distance = sum;
                    }
                    /* np.argmin chooses column zero even for an all-inf row.
                       Preserve that legacy corner case if limit is +inf. */
                    if (column == c_start || distance < best) {
                        best = distance;
                        best_id = candidate_id;
                    }
                }
                if (!(best <= limit)) best_id = -1;
                memcpy(out + (Py_ssize_t)query_id * 8, &best_id, 8);
            }
            pair_offset += (q_end - q_start) * width;
        }
    }
    Py_END_ALLOW_THREADS
    return output;
fail:
    PyMem_Free(seen);
    Py_XDECREF(output);
    return NULL;
}


/* Original slow-axis NumPy reductions: even zero-weight terms execute, in
 * candidate order. No contraction, reassociation or projected-variance rewrite. */
static void moments_row(const char *difference, const unsigned char *support,
                        int columns, double mean[2], double values[4])
{
    int column, count = 0, i, j;
    volatile double sum_x = 0.0, sum_y = 0.0;
    double denominator;
    for (column = 0; column < columns; ++column) {
        double weight = (double)support[column];
        volatile double x = nn_f64(difference, (Py_ssize_t)column * 2) * weight;
        volatile double y = nn_f64(difference, (Py_ssize_t)column * 2 + 1) * weight;
        sum_x = sum_x + x;
        sum_y = sum_y + y;
        count += support[column];
    }
    denominator = count > 0 ? (double)count : 1.0;
    mean[0] = sum_x / denominator;
    mean[1] = sum_y / denominator;
    for (i = 0; i < 2; ++i) {
        for (j = 0; j < 2; ++j) {
            volatile double sum = 0.0;
            for (column = 0; column < columns; ++column) {
                double weight = (double)support[column];
                volatile double x = nn_f64(difference, (Py_ssize_t)column * 2 + i) - mean[i];
                volatile double y = nn_f64(difference, (Py_ssize_t)column * 2 + j) - mean[j];
                volatile double product = x * y;
                volatile double weighted = product * weight;
                sum = sum + weighted;
            }
            values[i * 2 + j] = sum / denominator;
        }
    }
}

/* Per-row PCA moments only. Eigenvectors, validity gates, cell/candidate order,
 * radius selection and graph connectivity remain owned by the Python caller.
 * Native-endian f64 NxKx2 differences / uint8 NxK support are immutable bytes.
 * Capacity bounds imply at most 139,264,000 input bytes and 192,000 output
 * bytes. No dense centered/weighted floating-point temporaries are allocated. */
static PyObject *local_moments(PyObject *self, PyObject *args)
{
    PyObject *difference_obj, *support_obj, *rows_obj, *columns_obj;
    PyObject *means_obj = NULL, *covariance_obj = NULL, *result = NULL;
    const char *difference, *support;
    char *means, *covariance;
    int rows, columns, row;
    Py_ssize_t pairs, position;
    (void)self;
    if (!PyArg_ParseTuple(args, "OOOO", &difference_obj, &support_obj, &rows_obj, &columns_obj))
        return NULL;
    /* Bounds precede every product, including zero-dimension inputs. Products
       fit even a 32-bit Py_ssize_t; bool and arbitrarily large ints reject. */
    if (!bounded_int(rows_obj, "rows", MAX_QUERIES, &rows) ||
        !bounded_int(columns_obj, "columns", MAX_CANDIDATES, &columns))
        return NULL;
    pairs = (Py_ssize_t)rows * columns;
    if (!exact_bytes(difference_obj, pairs * 16, "differences") ||
        !exact_bytes(support_obj, pairs, "support"))
        return NULL;
    difference = PyBytes_AS_STRING(difference_obj);
    support = PyBytes_AS_STRING(support_obj);
    for (position = 0; position < pairs; ++position) {
        double x = nn_f64(difference, position * 2);
        double y = nn_f64(difference, position * 2 + 1);
        if (!isfinite(x) || !isfinite(y) || fabs(x) > 2e7 || fabs(y) > 2e7) {
            PyErr_SetString(PyExc_ValueError, "differences must be finite and bounded by 2e7");
            return NULL;
        }
        if ((unsigned char)support[position] > 1) {
            PyErr_SetString(PyExc_ValueError, "support values must be 0 or 1");
            return NULL;
        }
    }
    means_obj = PyBytes_FromStringAndSize(NULL, (Py_ssize_t)rows * 16);
    if (!means_obj) goto fail;
    covariance_obj = PyBytes_FromStringAndSize(NULL, (Py_ssize_t)rows * 32);
    if (!covariance_obj) goto fail;
    means = PyBytes_AS_STRING(means_obj);
    covariance = PyBytes_AS_STRING(covariance_obj);
    /* Input references are retained by args; outputs are not exposed. Separate
       volatile operations forbid contraction/reassociation. Even zero-weight
       terms execute in original candidate order, including signed zeros. */
    Py_BEGIN_ALLOW_THREADS
    for (row = 0; row < rows; ++row) {
        Py_ssize_t start = (Py_ssize_t)row * columns;
        double mean[2], values[4];
        moments_row(difference + start * 16, (const unsigned char *)support + start,
                    columns, mean, values);
        memcpy(means + (Py_ssize_t)row * 16, mean, 16);
        memcpy(covariance + (Py_ssize_t)row * 32, values, 32);
    }
    Py_END_ALLOW_THREADS
    result = PyTuple_Pack(2, means_obj, covariance_obj);
fail:
    Py_XDECREF(means_obj);
    Py_XDECREF(covariance_obj);
    return result;
}


/* One-radius local geometry. CSR preserves the caller's cell/candidate order;
 * only one Kx2 scratch row is needed, never a QxK floating-point tensor. All
 * validation and allocation precede GIL release. The caller still owns the
 * original 256-row eigensolver batches, smallest-radius selection and gates. */
typedef struct {
    SupportWork support;
    double points[MAX_POINTS * 2];
    int labels[MAX_POINTS];
    double differences[MAX_CANDIDATES * 2];
    unsigned char allowed[MAX_CANDIDATES];
    unsigned char connected[MAX_CANDIDATES];
    int seen[MAX_POINTS];
} GeometryWork;

static PyObject *local_geometry(PyObject *self, PyObject *args)
{
    PyObject *capsule, *size, *points_obj, *labels_obj, *parents_obj;
    PyObject *qo_obj, *qi_obj, *co_obj, *ci_obj, *limit_obj, *minimum_obj;
    PyObject *counts_obj = NULL, *means_obj = NULL, *covariance_obj = NULL, *result = NULL;
    const char *qo, *qi, *co, *ci;
    char *counts, *means, *covariance;
    Py_ssize_t nqo, nqi, nco, nci, g, k, groups;
    int64_t previous_q = 0, previous_c = 0;
    int minimum;
    double limit;
    Graph graph;
    GeometryWork *work = NULL;
    (void)self;
    if (!PyArg_ParseTuple(args, "OOOOOOOOOOO:local_geometry", &capsule, &size,
            &points_obj, &labels_obj, &parents_obj, &qo_obj, &qi_obj, &co_obj,
            &ci_obj, &limit_obj, &minimum_obj))
        return NULL;
    if (!PyCapsule_CheckExact(capsule)) {
        PyErr_SetString(PyExc_TypeError, "local_geometry requires a prepared graph capsule");
        return NULL;
    }
    if (!parse_graph(capsule, size, &graph) ||
        !bounded_int(minimum_obj, "minimum_points", MAX_CANDIDATES, &minimum))
        return NULL;
    if (!PyFloat_CheckExact(limit_obj)) {
        PyErr_SetString(PyExc_TypeError, "radius_squared must be a built-in float");
        return NULL;
    }
    limit = PyFloat_AS_DOUBLE(limit_obj);
    if (Py_IS_NAN(limit) || limit < 0.0) {
        PyErr_SetString(PyExc_ValueError, "radius_squared must be nonnegative and not NaN");
        return NULL;
    }
    if (!exact_bytes(points_obj, (Py_ssize_t)graph.size * 16, "points") ||
        !exact_bytes(labels_obj, (Py_ssize_t)graph.size * 8, "component_labels") ||
        !exact_bytes(parents_obj, (Py_ssize_t)graph.size * 8, "parents") ||
        !nn_bytes(qo_obj, 8, &qo, &nqo, "query_offsets") ||
        !nn_bytes(qi_obj, 8, &qi, &nqi, "query_ids") ||
        !nn_bytes(co_obj, 8, &co, &nco, "candidate_offsets") ||
        !nn_bytes(ci_obj, 8, &ci, &nci, "candidate_ids"))
        return NULL;
    /* These bounds also keep every byte product inside 32-bit Py_ssize_t. */
    if (nqo < 1 || nqo > MAX_POINTS + 1 || nco != nqo ||
        nqi > MAX_QUERIES || nci > (Py_ssize_t)MAX_POINTS * MAX_CANDIDATES) {
        PyErr_SetString(PyExc_ValueError, "bounded local geometry dimensions violated");
        return NULL;
    }
    groups = nqo - 1;
    if (nn_i64(qo, 0) != 0 || nn_i64(co, 0) != 0 ||
        nn_i64(qo, groups) != nqi || nn_i64(co, groups) != nci) {
        PyErr_SetString(PyExc_ValueError, "CSR offsets must start at zero and end at the ID count");
        return NULL;
    }
    for (g = 0; g < groups; ++g) {
        int64_t end_q = nn_i64(qo, g + 1), end_c = nn_i64(co, g + 1);
        if (end_q < previous_q || end_q > nqi || end_c < previous_c ||
            end_c > nci || end_c - previous_c > MAX_CANDIDATES) {
            PyErr_SetString(PyExc_ValueError, "CSR offsets must be monotonic and each patch at most 2048");
            return NULL;
        }
        previous_q = end_q;
        previous_c = end_c;
    }
    work = (GeometryWork *)PyMem_Malloc(sizeof(GeometryWork));
    if (!work) return PyErr_NoMemory();
    memcpy(work->points, PyBytes_AS_STRING(points_obj), (size_t)graph.size * 16);
    for (k = 0; k < graph.size * 2; ++k) {
        if (!Py_IS_FINITE(work->points[k]) || fabs(work->points[k]) > 1e7) {
            PyErr_SetString(PyExc_ValueError, "points must be finite and bounded by 1e7");
            goto fail;
        }
    }
    if (!copy_support_ids(labels_obj, graph.size, graph.size, "component_labels", work->labels) ||
        !validate_support_forest(&graph, parents_obj, &work->support))
        goto fail;
    memset(work->seen, 0, sizeof(work->seen));
    for (k = 0; k < nqi; ++k) {
        int64_t id = nn_i64(qi, k);
        if (id < 0 || id >= graph.size || work->seen[id]) {
            PyErr_SetString(PyExc_ValueError, "query IDs must be in [0, n) and globally unique");
            goto fail;
        }
        work->seen[id] = 1;
    }
    /* Epoch marks detect duplicates within each group, permitting reuse across
       groups, including groups with no queries. No unvalidated IDs survive. */
    memset(work->seen, 0, sizeof(work->seen));
    for (g = 0; g < groups; ++g) {
        for (k = (Py_ssize_t)nn_i64(co, g); k < nn_i64(co, g + 1); ++k) {
            int64_t id = nn_i64(ci, k);
            if (id < 0 || id >= graph.size || work->seen[id] == g + 1) {
                PyErr_SetString(PyExc_ValueError, "candidate IDs must be in [0, n) and unique within each group");
                goto fail;
            }
            work->seen[id] = (int)g + 1;
        }
    }
    counts_obj = PyBytes_FromStringAndSize(NULL, nqi * 8);
    means_obj = PyBytes_FromStringAndSize(NULL, nqi * 16);
    covariance_obj = PyBytes_FromStringAndSize(NULL, nqi * 32);
    if (!counts_obj || !means_obj || !covariance_obj) goto fail;
    counts = PyBytes_AS_STRING(counts_obj);
    means = PyBytes_AS_STRING(means_obj);
    covariance = PyBytes_AS_STRING(covariance_obj);
    memset(means, 0, (size_t)nqi * 16);
    memset(covariance, 0, (size_t)nqi * 32);
    /* args retain every immutable byte object and graph capsule. Pure C only
       below: no Python API, allocation, borrowed mutable buffers or callbacks. */
    Py_BEGIN_ALLOW_THREADS
    for (g = 0; g < groups; ++g) {
        Py_ssize_t begin = (Py_ssize_t)nn_i64(co, g);
        int columns = (int)(nn_i64(co, g + 1) - begin), column;
        for (column = 0; column < columns; ++column) {
            int id = (int)nn_i64(ci, begin + column);
            work->support.candidates[column] = id;
            work->support.positions[id] = column;
        }
        for (column = 0; column < columns; ++column) {
            int parent = work->support.parents[work->support.candidates[column]];
            work->support.parent_positions[column] = parent < 0 ? -1 : work->support.positions[parent];
        }
        for (k = (Py_ssize_t)nn_i64(qo, g); k < nn_i64(qo, g + 1); ++k) {
            int id = (int)nn_i64(qi, k);
            int64_t count;
            work->support.targets[0] = id;
            for (column = 0; column < columns; ++column) {
                int candidate = work->support.candidates[column];
                volatile double dx = work->points[2 * candidate] - work->points[2 * id];
                volatile double dy = work->points[2 * candidate + 1] - work->points[2 * id + 1];
                volatile double xx = dx * dx, yy = dy * dy;
                volatile double squared = xx + yy;
                work->differences[2 * column] = dx;
                work->differences[2 * column + 1] = dy;
                work->allowed[column] = (unsigned char)(squared <= limit &&
                    work->labels[candidate] == work->labels[id]);
            }
            support_rows(&graph, &work->support, columns, 1, work->allowed,
                         work->connected, (unsigned char *)(counts + k * 8));
            memcpy(&count, counts + k * 8, 8);
            if (count >= minimum) {
                double mean[2], values[4];
                moments_row((const char *)work->differences, work->connected, columns, mean, values);
                memcpy(means + k * 16, mean, 16);
                memcpy(covariance + k * 32, values, 32);
            }
        }
        /* Clear every position, including query-free groups. Otherwise an old
           candidate could falsely certify a seed or parent in the next group. */
        for (column = 0; column < columns; ++column)
            work->support.positions[work->support.candidates[column]] = -1;
    }
    Py_END_ALLOW_THREADS
    result = PyTuple_Pack(3, counts_obj, means_obj, covariance_obj);
fail:
    Py_XDECREF(counts_obj);
    Py_XDECREF(means_obj);
    Py_XDECREF(covariance_obj);
    PyMem_Free(work);
    return result;
}

/* Same scalar operation order and outward binary64 rounding as
 * finite_support.projection_bounds. No fused dot product, epsilon, or changed
 * support extent. The caller has already validated its immutable coordinates.
 * Keep volatile stores so compilers cannot contract or retain extra precision.
 */
static int finite_outward(double value, double *lower, double *upper)
{
    if (!isfinite(value)) {
        PyErr_SetString(PyExc_ValueError, "nonfinite interval arithmetic");
        return 0;
    }
    *lower = nextafter(value, -INFINITY);
    *upper = nextafter(value, INFINITY);
    if (!isfinite(*lower) || !isfinite(*upper)) {
        PyErr_SetString(PyExc_ValueError, "unbounded interval arithmetic");
        return 0;
    }
    return 1;
}

static int finite_projection_part(double point, double origin, double tangent,
                                  double *lower, double *upper)
{
    double a, b, discard;
    volatile double subtraction, first, second;
    if (point == origin) {
        a = b = 0.;
    } else {
        subtraction = point - origin;
        if (!finite_outward(subtraction, &a, &b)) return 0;
    }
    if (tangent == 0. || (a == 0. && b == 0.)) {
        *lower = *upper = 0.;
        return 1;
    }
    first = (tangent > 0. ? a : b) * tangent;
    second = (tangent > 0. ? b : a) * tangent;
    if (!finite_outward(first, lower, &discard)) return 0;
    return finite_outward(second, &discard, upper);
}

static PyObject *finite_projection_bounds(PyObject *self, PyObject *args)
{
    double px, ox, tx, py, oy, ty, al, au, bl, bu, lower, upper, discard;
    volatile double first, second;
    if (!PyArg_ParseTuple(args, "dddddd:finite_projection_bounds",
                          &px, &ox, &tx, &py, &oy, &ty)) return NULL;
    if (!(isfinite(px) && isfinite(ox) && isfinite(tx) &&
          isfinite(py) && isfinite(oy) && isfinite(ty))) {
        PyErr_SetString(PyExc_ValueError, "finite projection requires finite inputs");
        return NULL;
    }
    if (!finite_projection_part(px, ox, tx, &al, &au) ||
        !finite_projection_part(py, oy, ty, &bl, &bu)) return NULL;
    if (al == 0. && au == 0.) return Py_BuildValue("(dd)", bl, bu);
    if (bl == 0. && bu == 0.) return Py_BuildValue("(dd)", al, au);
    first = al + bl;
    second = au + bu;
    if (!finite_outward(first, &lower, &discard) ||
        !finite_outward(second, &discard, &upper)) return NULL;
    return Py_BuildValue("(dd)", lower, upper);
}

/* Stateless current-frame union. Python retains signed-normal admission,
 * source/vote/state validation; this kernel preserves exact endpoint identity
 * and the existing outward IEEE projection arithmetic. */
#define CU_MAX_SEGMENTS 200000
#define CU_COORDINATE 1.0e7

typedef struct {
    double x, y, lower, upper;
} CuVertex;

typedef struct {
    int first, second;
    unsigned char reliable;
} CuEdge;

typedef struct {
    double value;
    int total, reliable;
    unsigned int ordinal;
} CuEvent;

typedef struct {
    double lower, upper;
    unsigned char reliable;
} CuPiece;

/* Hash equality is numeric equality for finite doubles, like Python tuples.
 * Normalize zero ONLY for hashing; the first admitted coordinate's original
 * double is retained and used for projection. No spatial quantization. */
static uint64_t cu_bits(double value)
{
    uint64_t bits;
    if (value == 0.0)
        return UINT64_C(0);
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static uint64_t cu_mix(uint64_t value)
{
    value ^= value >> 30;
    value *= UINT64_C(0xbf58476d1ce4e5b9);
    value ^= value >> 27;
    value *= UINT64_C(0x94d049bb133111eb);
    return value ^ (value >> 31);
}

/* Exactly the existing eight-API projection arithmetic, without constructing
 * a per-vertex Python tuple. This helper MUST run with the GIL: existing
 * finite_projection_part/finite_outward set Python exceptions on failure. */
static int cu_projection(double x, double y, double ox, double oy,
                         double tx, double ty, double *lower, double *upper)
{
    double al, au, bl, bu, discard;
    volatile double first, second;
    if (!finite_projection_part(x, ox, tx, &al, &au) ||
            !finite_projection_part(y, oy, ty, &bl, &bu))
        return 0;
    if (al == 0.0 && au == 0.0) {
        *lower = bl; *upper = bu;
        return 1;
    }
    if (bl == 0.0 && bu == 0.0) {
        *lower = al; *upper = au;
        return 1;
    }
    first = al + bl;
    second = au + bu;
    return finite_outward(first, lower, &discard) &&
           finite_outward(second, &discard, upper);
}

static int cu_root(int *parents, int vertex)
{
    while (parents[vertex] != vertex) {
        parents[vertex] = parents[parents[vertex]];
        vertex = parents[vertex];
    }
    return vertex;
}

static void cu_join(int *parents, unsigned char *ranks, int first, int second)
{
    int a = cu_root(parents, first), b = cu_root(parents, second), swap;
    if (a == b)
        return;
    if (ranks[a] < ranks[b]) {
        swap = a; a = b; b = swap;
    }
    parents[b] = a;
    if (ranks[a] == ranks[b])
        ++ranks[a];
}

/* Preserve Python dict insertion order for component creation, and Python
 * min/max's first-value behavior on equal values, including signed zero. */
static void cu_events(const CuVertex *vertices, int *parents, const int *order,
                      int count, int trusted, double *lower, double *upper,
                      unsigned char *seen, int *root_order,
                      CuEvent *events, int *event_count)
{
    int index, vertex, owner, roots = 0;
    for (index = 0; index < count; ++index) {
        vertex = order ? order[index] : index;
        owner = cu_root(parents, vertex);
        if (!seen[owner]) {
            seen[owner] = 1;
            root_order[roots++] = owner;
            lower[owner] = vertices[vertex].upper;
            upper[owner] = vertices[vertex].lower;
        } else {
            if (vertices[vertex].upper < lower[owner])
                lower[owner] = vertices[vertex].upper;
            if (vertices[vertex].lower > upper[owner])
                upper[owner] = vertices[vertex].lower;
        }
    }
    for (index = 0; index < roots; ++index) {
        owner = root_order[index];
        if (lower[owner] < upper[owner]) {
            CuEvent *start = &events[*event_count];
            start->value = lower[owner];
            start->total = 1; start->reliable = trusted;
            start->ordinal = (unsigned int)(*event_count);
            ++*event_count;
            start = &events[*event_count];
            start->value = upper[owner];
            start->total = -1; start->reliable = -trusted;
            start->ordinal = (unsigned int)(*event_count);
            ++*event_count;
        }
        seen[owner] = 0;
    }
}

static int cu_event_compare(const void *first, const void *second)
{
    const CuEvent *a = (const CuEvent *)first, *b = (const CuEvent *)second;
    if (a->value < b->value) return -1;
    if (a->value > b->value) return 1;
    /* qsort itself is not stable. Explicit ordinal keeps the first Python
     * event-key representative when -0.0 and +0.0 compare equal. */
    return a->ordinal < b->ordinal ? -1 : a->ordinal > b->ordinal ? 1 : 0;
}

static PyObject *finite_current_union(PyObject *self, PyObject *args)
{
    PyObject *segments_obj, *admitted_obj, *reliable_obj, *count_obj;
    PyObject *ox_obj, *oy_obj, *tx_obj, *ty_obj, *output = NULL;
    PyObject *scalars[4];
    const char *segments;
    const unsigned char *admitted, *reliable;
    CuVertex *vertices = NULL;
    CuEdge *edges = NULL;
    CuEvent *events = NULL;
    CuPiece *pieces = NULL;
    int *table = NULL, *base_parents = NULL, *trusted_parents = NULL;
    int *trusted_order = NULL, *root_order = NULL;
    unsigned char *base_ranks = NULL, *trusted_ranks = NULL, *seen = NULL;
    double *lower = NULL, *upper = NULL;
    double values[4], ox, oy, tx, ty;
    int count, admitted_count = 0, capacity, vertex_count = 0, edge_count = 0, trusted_count = 0;
    int trusted_vertices = 0, event_count = 0, piece_count = 0;
    int index, endpoint, slot, vertex, first, stop, active, trusted_active;
    size_t table_size = 1, table_mask;
    (void)self;
    if (!PyArg_ParseTuple(args, "OOOOOOOO:finite_current_union",
            &segments_obj, &admitted_obj, &reliable_obj, &count_obj,
            &ox_obj, &oy_obj, &tx_obj, &ty_obj))
        return NULL;
    if (!bounded_int(count_obj, "count", CU_MAX_SEGMENTS, &count) ||
            !exact_bytes(segments_obj, (Py_ssize_t)count * 32, "segments") ||
            !exact_bytes(admitted_obj, count, "admitted") ||
            !exact_bytes(reliable_obj, count, "reliable"))
        return NULL;
    scalars[0] = ox_obj; scalars[1] = oy_obj;
    scalars[2] = tx_obj; scalars[3] = ty_obj;
    for (index = 0; index < 4; ++index) {
        if (!PyFloat_CheckExact(scalars[index])) {
            PyErr_SetString(PyExc_TypeError, "chart scalars must be built-in floats");
            return NULL;
        }
        values[index] = PyFloat_AS_DOUBLE(scalars[index]);
        if (!isfinite(values[index])) {
            PyErr_SetString(PyExc_ValueError, "chart scalars must be finite");
            return NULL;
        }
    }
    ox = values[0]; oy = values[1]; tx = values[2]; ty = values[3];
    /* Public Python owns the exact unit-norm admission. This FFI guard is a
     * necessary coordinate bound, not a second platform-specific hypot gate. */
    if (fabs(ox) > CU_COORDINATE || fabs(oy) > CU_COORDINATE ||
            fabs(tx) > 1.0 + 64.0 * DBL_EPSILON ||
            fabs(ty) > 1.0 + 64.0 * DBL_EPSILON) {
        PyErr_SetString(PyExc_ValueError, "chart scalars exceed validated bounds");
        return NULL;
    }
    segments = PyBytes_AS_STRING(segments_obj);
    admitted = (const unsigned char *)PyBytes_AS_STRING(admitted_obj);
    reliable = (const unsigned char *)PyBytes_AS_STRING(reliable_obj);
    /* Validate all input bytes, even for a non-admitted segment, before work.
     * No mutable buffer/view, unaligned cast or external owner is retained. */
    for (index = 0; index < count; ++index) {
        for (endpoint = 0; endpoint < 4; ++endpoint) {
            double coordinate = nn_f64(segments, (Py_ssize_t)index * 4 + endpoint);
            if (!isfinite(coordinate) || fabs(coordinate) > CU_COORDINATE) {
                PyErr_SetString(PyExc_ValueError, "segments must be finite and bounded");
                return NULL;
            }
        }
        if (admitted[index] > 1 || reliable[index] > 1) {
            PyErr_SetString(PyExc_ValueError, "admission/reliability bytes must be zero or one");
            return NULL;
        }
        admitted_count += admitted[index];
    }
    /* Every declared coordinate and both masks have now been checked.
     * In particular, rejected rows cannot hide malformed input. Size private
     * graph/event scratch from admitted rows only, preserving their order. */
    if (!admitted_count)
        return PyTuple_New(0);
    capacity = admitted_count * 2;
    while (table_size < (size_t)capacity * 2)
        table_size <<= 1;
    table_mask = table_size - 1;
    vertices = (CuVertex *)PyMem_Malloc((size_t)capacity * sizeof(*vertices));
    edges = (CuEdge *)PyMem_Malloc((size_t)admitted_count * sizeof(*edges));
    events = (CuEvent *)PyMem_Malloc((size_t)admitted_count * 4 * sizeof(*events));
    pieces = (CuPiece *)PyMem_Malloc((size_t)admitted_count * 4 * sizeof(*pieces));
    table = (int *)PyMem_Malloc(table_size * sizeof(*table));
    base_parents = (int *)PyMem_Malloc((size_t)capacity * sizeof(*base_parents));
    trusted_parents = (int *)PyMem_Malloc((size_t)capacity * sizeof(*trusted_parents));
    trusted_order = (int *)PyMem_Malloc((size_t)capacity * sizeof(*trusted_order));
    root_order = (int *)PyMem_Malloc((size_t)capacity * sizeof(*root_order));
    base_ranks = (unsigned char *)PyMem_Calloc((size_t)capacity, 1);
    trusted_ranks = (unsigned char *)PyMem_Calloc((size_t)capacity, 1);
    seen = (unsigned char *)PyMem_Calloc((size_t)capacity, 1);
    lower = (double *)PyMem_Malloc((size_t)capacity * sizeof(*lower));
    upper = (double *)PyMem_Malloc((size_t)capacity * sizeof(*upper));
    if (!vertices || !edges || !events || !pieces || !table || !base_parents ||
            !trusted_parents || !trusted_order || !root_order || !base_ranks ||
            !trusted_ranks || !seen || !lower || !upper) {
        PyErr_NoMemory();
        goto cleanup;
    }
    for (slot = 0; slot < (int)table_size; ++slot)
        table[slot] = -1;
    for (index = 0; index < capacity; ++index)
        trusted_parents[index] = -1;
    /* Keep the GIL for projection helpers which use PyErr on arithmetic error. */
    for (index = 0; index < count; ++index) {
        int ids[2];
        if (!admitted[index])
            continue;
        for (endpoint = 0; endpoint < 2; ++endpoint) {
            double x = nn_f64(segments, (Py_ssize_t)index * 4 + endpoint * 2);
            double y = nn_f64(segments, (Py_ssize_t)index * 4 + endpoint * 2 + 1);
            uint64_t hash = cu_mix(cu_bits(x)) ^ cu_mix(cu_bits(y) + UINT64_C(0x9e3779b97f4a7c15));
            slot = (int)(hash & table_mask);
            while ((vertex = table[slot]) >= 0 &&
                    !(vertices[vertex].x == x && vertices[vertex].y == y))
                slot = (int)(((size_t)slot + 1) & table_mask);
            if (vertex < 0) {
                vertex = vertex_count++;
                vertices[vertex].x = x; vertices[vertex].y = y;
                if (!cu_projection(x, y, ox, oy, tx, ty,
                        &vertices[vertex].lower, &vertices[vertex].upper))
                    goto cleanup;
                table[slot] = vertex;
                base_parents[vertex] = vertex;
            }
            ids[endpoint] = vertex;
            if (reliable[index] && trusted_parents[vertex] < 0) {
                trusted_parents[vertex] = vertex;
                trusted_order[trusted_vertices++] = vertex;
            }
        }
        edges[edge_count].first = ids[0]; edges[edge_count].second = ids[1];
        edges[edge_count].reliable = reliable[index];
        ++edge_count;
        trusted_count += reliable[index];
    }
    /* Everything below is bounded private memory and has no Python callbacks,
     * allocation, projection error path or mutable input access. */
    Py_BEGIN_ALLOW_THREADS
    for (index = 0; index < edge_count; ++index) {
        cu_join(base_parents, base_ranks, edges[index].first, edges[index].second);
        if (edges[index].reliable)
            cu_join(trusted_parents, trusted_ranks, edges[index].first, edges[index].second);
    }
    cu_events(vertices, base_parents, NULL, vertex_count,
              edge_count == trusted_count, lower, upper, seen, root_order, events, &event_count);
    if (trusted_count && trusted_count != edge_count)
        cu_events(vertices, trusted_parents, trusted_order, trusted_vertices,
                  1, lower, upper, seen, root_order, events, &event_count);
    qsort(events, (size_t)event_count, sizeof(*events), cu_event_compare);
    active = trusted_active = 0;
    for (first = 0; first < event_count; first = stop) {
        int total_delta = 0, reliable_delta = 0;
        double lo = events[first].value, hi;
        unsigned char trusted;
        stop = first;
        while (stop < event_count && events[stop].value == lo) {
            total_delta += events[stop].total;
            reliable_delta += events[stop].reliable;
            ++stop;
        }
        active += total_delta; trusted_active += reliable_delta;
        if (stop == event_count || !active)
            continue;
        hi = events[stop].value;
        trusted = trusted_active > 0;
        if (piece_count && pieces[piece_count - 1].upper == lo &&
                pieces[piece_count - 1].reliable == trusted) {
            pieces[piece_count - 1].upper = hi;
        } else {
            pieces[piece_count].lower = lo; pieces[piece_count].upper = hi;
            pieces[piece_count].reliable = trusted;
            ++piece_count;
        }
    }
    Py_END_ALLOW_THREADS
    output = PyTuple_New(piece_count);
    if (!output)
        goto cleanup;
    for (index = 0; index < piece_count; ++index) {
        PyObject *piece = Py_BuildValue("(ddO)", pieces[index].lower, pieces[index].upper,
                                       pieces[index].reliable ? Py_True : Py_False);
        if (!piece) {
            Py_CLEAR(output);
            goto cleanup;
        }
        PyTuple_SET_ITEM(output, index, piece);
    }
cleanup:
    PyMem_Free(vertices); PyMem_Free(edges); PyMem_Free(events); PyMem_Free(pieces);
    PyMem_Free(table); PyMem_Free(base_parents); PyMem_Free(trusted_parents);
    PyMem_Free(trusted_order); PyMem_Free(root_order); PyMem_Free(base_ranks);
    PyMem_Free(trusted_ranks); PyMem_Free(seen); PyMem_Free(lower); PyMem_Free(upper);
    return output;
}

static PyMethodDef methods[] = {
    {"finite_current_union", finite_current_union, METH_VARARGS,
     "Exact current-frame finite union from immutable f64 segments and caller-admitted masks."},
    {"finite_projection_bounds", finite_projection_bounds, METH_VARARGS,
     "Exact outward projection enclosure for six finite f64 scalars, ordered px,ox,tx,py,oy,ty."},
    {"local_geometry", local_geometry, METH_VARARGS,
     "Bounded CSR local support and original PCA moments; caller owns radii, eigensolver and gates."},
    {"local_moments", local_moments, METH_VARARGS,
     "Bounded per-row mean/covariance (f64 differences, uint8 support, rows, columns); no PCA gates or eigensolver."},
    {"nearest_grouped", nearest_grouped, METH_VARARGS,
     "Exact grouped distance/argmin, retaining caller-provided normal compatibility and candidate order."},
    {"cross_section_weights", cross_section_weights, METH_VARARGS,
     "Exact current tangent-slab connectivity weights; duplicate reliable IDs retain multiplicity."},
    {"local_support", local_support, METH_VARARGS,
     "Exact local support (graph, n, int64 candidates, int64 targets, uint8 allowed, int64 parents). Returns uint8 support and native-int64 counts bytes."},
    {"radius_graph", radius_graph, METH_VARARGS,
     "Exact adjacent-key graph (f64 points, int64 keys, n, radius_squared float)."},
    {"prepare_graph", prepare_graph, METH_VARARGS,
     "Copy packed bytes into an immutable owned graph capsule (n<=4000)."},
    {"connected_subsets", connected_subsets, METH_VARARGS,
     "Exact seed reachability within each allowed packed mask (n, queries <=4000)."},
    {"graph_forest", graph_forest, METH_VARARGS,
     "Canonical lowest-bit components and first-discovery parents (n<=4000)."},
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef module = {
    PyModuleDef_HEAD_INIT, "_lane_mapping_lya_native", "Bounded exact lane graph kernels.", -1, methods
};

PyMODINIT_FUNC PyInit__lane_mapping_lya_native(void)
{
    PyObject *result = PyModule_Create(&module);
    if (!result)
        return NULL;
    if (PyModule_AddIntConstant(result, "API_VERSION", 1) < 0 ||
        PyModule_AddIntConstant(result, "RADIUS_GRAPH_API_VERSION", 1) < 0 ||
        PyModule_AddIntConstant(result, "LOCAL_SUPPORT_API_VERSION", 1) < 0 ||
        PyModule_AddIntConstant(result, "NEAREST_API_VERSION", 1) < 0 ||
        PyModule_AddIntConstant(result, "CROSS_SECTION_WEIGHTS_API_VERSION", 1) < 0 ||
        PyModule_AddIntConstant(result, "LOCAL_MOMENTS_API_VERSION", 1) < 0 ||
        PyModule_AddIntConstant(result, "LOCAL_GEOMETRY_API_VERSION", 1) < 0 ||
        PyModule_AddIntConstant(result, "FINITE_PROJECTION_BOUNDS_API_VERSION", 1) < 0 ||
        PyModule_AddIntConstant(result, "FINITE_CURRENT_UNION_API_VERSION", 1) < 0 ||
        PyModule_AddIntConstant(result, "MAX_CANDIDATES", MAX_CANDIDATES) < 0 ||
        PyModule_AddIntConstant(result, "MAX_POINTS", MAX_POINTS) < 0 ||
        PyModule_AddIntConstant(result, "MAX_QUERIES", MAX_QUERIES) < 0 ||
        PyModule_AddStringConstant(result, "SOURCE_SHA256", SOURCE_SHA256) < 0) {
        Py_DECREF(result);
        return NULL;
    }
    return result;
}
