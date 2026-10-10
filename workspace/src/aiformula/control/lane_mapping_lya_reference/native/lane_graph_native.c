#define PY_SSIZE_T_CLEAN
#include <Python.h>
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
#define MAX_WORDS ((MAX_POINTS + 63) / 64)
#define GRAPH_CAPSULE "lane_mapping_lya_native.Graph.v1"

/* Diagnostic prototype: packed rows use little-endian bit order, independently
 * of host alignment/endianness. No geometry or floating-point gate lives here. */
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

static PyMethodDef methods[] = {
    {"prepare_graph", prepare_graph, METH_VARARGS,
     "Copy packed bytes into an immutable owned graph capsule (n<=4000)."},
    {"connected_subsets", connected_subsets, METH_VARARGS,
     "Exact seed reachability within each allowed packed mask (n, queries <=4000)."},
    {"graph_forest", graph_forest, METH_VARARGS,
     "Canonical lowest-bit components and first-discovery parents (n<=4000)."},
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef module = {
    PyModuleDef_HEAD_INIT, "_lane_mapping_lya_native", "Isolated exact topology prototype.", -1, methods
};

PyMODINIT_FUNC PyInit__lane_mapping_lya_native(void)
{
    PyObject *result = PyModule_Create(&module);
    if (!result)
        return NULL;
    if (PyModule_AddIntConstant(result, "API_VERSION", 1) < 0 ||
        PyModule_AddIntConstant(result, "MAX_POINTS", MAX_POINTS) < 0 ||
        PyModule_AddIntConstant(result, "MAX_QUERIES", MAX_QUERIES) < 0 ||
        PyModule_AddStringConstant(result, "SOURCE_SHA256", SOURCE_SHA256) < 0) {
        Py_DECREF(result);
        return NULL;
    }
    return result;
}
