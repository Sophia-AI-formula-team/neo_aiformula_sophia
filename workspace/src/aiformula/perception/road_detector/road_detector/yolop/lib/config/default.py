import os

from yacs.config import CfgNode as CN


_C = CN()

_C.LOG_DIR = "runs/"
_C.GPUS = (0,)
# Recommended default: use up to 8 workers or the available CPU cores, whichever is smaller.
_C.WORKERS = min(8, os.cpu_count() or 1)
_C.PIN_MEMORY = True
_C.PERSISTENT_WORKERS = True
_C.PREFETCH_FACTOR = 4
_C.CHANNELS_LAST = False
_C.ALLOW_TF32 = True
_C.PRINT_FREQ = 20
_C.AUTO_RESUME = False
_C.DEBUG = False

# CUDNN
_C.CUDNN = CN()
_C.CUDNN.BENCHMARK = True
_C.CUDNN.DETERMINISTIC = False
_C.CUDNN.ENABLED = True

# MODEL
_C.MODEL = CN(new_allowed=True)
_C.MODEL.NAME = "lane_only"
_C.MODEL.PRETRAINED = ""
_C.MODEL.IMAGE_SIZE = [640, 640]
_C.MODEL.EXTRA = CN(new_allowed=True)

# LOSS
_C.LOSS = CN(new_allowed=True)
_C.LOSS.LANE_BCE_WEIGHT = 1.0
_C.LOSS.LANE_DICE_WEIGHT = 1.0
_C.LOSS.LANE_POS_WEIGHT = 0.0
_C.LOSS.AUTO_POS_WEIGHT = True
_C.LOSS.POS_WEIGHT_SAMPLE_COUNT = 16
_C.LOSS.MIN_POS_WEIGHT = 1.0
_C.LOSS.MAX_POS_WEIGHT = 50.0
_C.LOSS.HARD_NEGATIVE_RATIO = 0.0
_C.LOSS.PRED_THRESHOLD = 0.5

# DATASET
_C.DATASET = CN(new_allowed=True)
_C.DATASET.ROOT = "dataset"
_C.DATASET.IMAGE_DIR = "images"
_C.DATASET.LANE_MASK_DIR = "lane_masks"
_C.DATASET.LANE_MASK_FORMAT = "png"
_C.DATASET.DATASET = "LaneDataset"
_C.DATASET.TRAIN_SET = "train"
_C.DATASET.TEST_SET = "val"
_C.DATASET.DATA_FORMAT = "jpg"
_C.DATASET.ORG_IMG_SIZE = [720, 1280]

# Lane-seg augmentations
_C.DATASET.FLIP = True
_C.DATASET.SCALE_FACTOR = 0.25
_C.DATASET.ROT_FACTOR = 10
_C.DATASET.TRANSLATE = 0.1
_C.DATASET.SHEAR = 0.0
_C.DATASET.HSV_H = 0.015
_C.DATASET.HSV_S = 0.7
_C.DATASET.HSV_V = 0.4

# TRAIN
_C.TRAIN = CN(new_allowed=True)
_C.TRAIN.LR0 = 0.001
_C.TRAIN.LRF = 0.2
_C.TRAIN.WARMUP_EPOCHS = 3.0
_C.TRAIN.WARMUP_BIASE_LR = 0.1
_C.TRAIN.WARMUP_MOMENTUM = 0.8
_C.TRAIN.OPTIMIZER = "adam"
_C.TRAIN.MOMENTUM = 0.937
_C.TRAIN.WD = 0.0005
_C.TRAIN.NESTEROV = True
_C.TRAIN.BEGIN_EPOCH = 0
_C.TRAIN.END_EPOCH = 240
_C.TRAIN.VAL_FREQ = 1
_C.TRAIN.BATCH_SIZE_PER_GPU = 24
_C.TRAIN.AUTO_BATCH = False
_C.TRAIN.AUTO_BATCH_MAX = 0
_C.TRAIN.AUTO_BATCH_STEP = 4
_C.TRAIN.PERF_LOG = False
_C.TRAIN.SHUFFLE = True

# TEST
_C.TEST = CN(new_allowed=True)
_C.TEST.BATCH_SIZE_PER_GPU = 24
_C.TEST.MODEL_FILE = ""
_C.TEST.PLOTS = True
_C.TEST.VIS_CONFIG = "vis.config"
_C.TEST.VIS_OUTPUT_DIR = "visualization"
_C.TEST.VIS_USE_FIXED_SET = True

# POSTPROCESS
_C.POST = CN(new_allowed=True)
_C.POST.ENABLED = True
_C.POST.VERTICAL_CLOSE = True
_C.POST.VERTICAL_KERNEL = [3, 11]
_C.POST.COMPONENT_FILTER = True
_C.POST.MIN_COMPONENT_AREA = 60
_C.POST.MIN_COMPONENT_HEIGHT = 24
_C.POST.MAX_COMPONENT_ASPECT_RATIO = 1.8
_C.POST.BOTTOM_TOUCH_MARGIN = 48
_C.POST.FP_WARNING_RATIO = 2.0


def update_config(cfg, args):
    cfg.defrost()

    if getattr(args, "modelDir", ""):
        cfg.OUTPUT_DIR = args.modelDir

    if getattr(args, "logDir", ""):
        cfg.LOG_DIR = args.logDir

    if getattr(args, "dataDir", ""):
        cfg.DATASET.ROOT = args.dataDir

    if getattr(args, "batch_size", 0):
        cfg.TRAIN.BATCH_SIZE_PER_GPU = args.batch_size

    if getattr(args, "workers", -1) >= 0:
        cfg.WORKERS = args.workers

    if getattr(args, "val_freq", 0):
        cfg.TRAIN.VAL_FREQ = args.val_freq

    if getattr(args, "prefetch_factor", 0):
        cfg.PREFETCH_FACTOR = args.prefetch_factor

    if getattr(args, "auto_batch", False):
        cfg.TRAIN.AUTO_BATCH = True

    if getattr(args, "auto_batch_max", 0):
        cfg.TRAIN.AUTO_BATCH_MAX = args.auto_batch_max

    if getattr(args, "auto_batch_step", 0):
        cfg.TRAIN.AUTO_BATCH_STEP = args.auto_batch_step

    if getattr(args, "perf_log", False):
        cfg.TRAIN.PERF_LOG = True

    cfg.freeze()
