# Limit OpenCV internal threads to avoid idle thread contention.
# Modules that need parallel cv2 ops can call cv2.setNumThreads() in start().
try:
    import cv2

    cv2.setNumThreads(2)
except ImportError:
    pass
