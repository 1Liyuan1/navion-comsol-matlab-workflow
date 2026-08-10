class UsbCamera:
    def __init__(self):
        self._capture = None
        self._cv2 = None

    @property
    def is_open(self):
        return self._capture is not None and self._capture.isOpened()

    def open(self, device_index):
        try:
            import cv2
        except ImportError as error:
            raise RuntimeError("Camera support requires opencv-python and Pillow. Run: python -m pip install -r requirements.txt") from error
        self.close()
        self._cv2 = cv2
        backend = getattr(cv2, "CAP_DSHOW", 0)
        self._capture = cv2.VideoCapture(device_index, backend)
        if not self._capture.isOpened():
            self._capture.release()
            self._capture = cv2.VideoCapture(device_index)
        if not self._capture.isOpened():
            self._capture = None
            raise RuntimeError("Cannot open USB camera device {0}.".format(device_index))
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def read_rgb(self):
        if not self.is_open:
            return None
        ok, frame = self._capture.read()
        if not ok:
            return None
        return self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)

    def close(self):
        if self._capture is not None:
            self._capture.release()
            self._capture = None
