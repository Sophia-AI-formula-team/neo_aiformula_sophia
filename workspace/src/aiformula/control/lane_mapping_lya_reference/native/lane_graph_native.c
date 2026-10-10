#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <float.h>
#include <stdint.h>
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

static PyMethodDef methods[] = {
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
        PyModule_AddIntConstant(result, "MAX_CANDIDATES", MAX_CANDIDATES) < 0 ||
        PyModule_AddIntConstant(result, "MAX_POINTS", MAX_POINTS) < 0 ||
        PyModule_AddIntConstant(result, "MAX_QUERIES", MAX_QUERIES) < 0 ||
        PyModule_AddStringConstant(result, "SOURCE_SHA256", SOURCE_SHA256) < 0) {
        Py_DECREF(result);
        return NULL;
    }
    return result;
}
