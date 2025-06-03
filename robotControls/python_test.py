#!/usr/bin/env python3
import threading
import queue
import time
import random

# Shared queue
q = queue.Queue()

# Producer thread
def producer():
    for i in range(10):
        item = 'item-{}'.format(i)
        print('Producing {}'.format(item))
        q.put(item)  # Put item into the queue
        time.sleep(random.uniform(0.1, 0.5))  # Simulate work

    q.put(None)  # Sentinel value to signal consumer to exit

# Consumer thread
def consumer():
    while True:
        item = q.get()  # Block until item is available
        if item is None:
            break  # Exit on sentinel
        print('Consuming {}'.format(item))
        time.sleep(random.uniform(0.1, 0.3))  # Simulate processing
    print('Consumer done.')

# Start threads
producer_thread = threading.Thread(target=producer)
consumer_thread = threading.Thread(target=consumer)

producer_thread.start()
consumer_thread.start()

producer_thread.join()
consumer_thread.join()

print('All done.')

