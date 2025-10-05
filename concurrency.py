from functools import wraps
import threading
from concurrent.futures import ThreadPoolExecutor
import time

def run_async(func):
    """Decorator to run a method asynchronously in its own thread."""
    @wraps(func)
    def wrapper(*args, **kwargs):
        thread = threading.Thread(target=func, args=args, kwargs=kwargs)
        thread.daemon = True
        thread.start()
        return thread
    return wrapper

class MultiThread:
    """Base class that enables methods to run asynchronously when decorated with @run_async."""
    def __init__(self):
        self.executor = ThreadPoolExecutor(max_workers=5)
        self._threads = []
        self.shutdown_event = threading.Event()
    
    def _track_thread(self, thread):
        """Track threads created by async methods."""
        self._threads.append(thread)
        return thread
    
    def wait_all(self, timeout=None):
        """Wait for all async threads to complete."""
        for thread in self._threads:
            if thread.is_alive():
                thread.join(timeout=timeout)

    def timer(self, interval, func, *args, **kwargs):
        """Run a function at regular intervals in a separate thread."""
        def loop():
            while not self.shutdown_event.is_set():
                time.sleep(interval)
                func(*args, **kwargs)
        thread = threading.Thread(target=loop)
        thread.daemon = True
        thread.start()
        return thread