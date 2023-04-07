import time


class LoopTracker:
    def __init__(self, sample_size: int = 200) -> None:
        self._sample_size = sample_size
        self._last_end_time = None
        self._loop_time_history = []
        self._fps_history = []

    def start_iteration(self) -> None:
        self._start_time = time.time()

    def stop_iteration(self) -> None:
        end_time = time.time()
        loop_time = (end_time - self._start_time) * 1000
        self._loop_time_history.append(loop_time)

        if self._last_end_time:
            fps = 1 / (end_time - self._last_end_time)
            self._fps_history.append(fps)
        self._last_end_time = end_time

        while len(self._loop_time_history) >= self._sample_size:
            self._loop_time_history.pop(0)
        while len(self._fps_history) >= self._sample_size:
            self._fps_history.pop(0)

    def last_loop_time(self) -> float:
        try:
            return self._loop_time_history[-1]
        except IndexError:
            return -1

    def last_fps(self) -> float:
        try:
            return self._fps_history[-1]
        except IndexError:
            return -1

    def mean_loop_time(self) -> float:
        try:
            return sum(self._loop_time_history) / len(self._loop_time_history)
        except ZeroDivisionError:
            return -1

    def mean_fps(self) -> float:
        try:
            return sum(self._fps_history) / len(self._fps_history)
        except ZeroDivisionError:
            return -1
