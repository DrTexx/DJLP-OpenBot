import _thread
import time

val1 = 1
val2 = 1

def th_func_add(delay, t_id):
    global val1
    while True:
        time.sleep(delay)
        val1 = val1 + 1
def th_func_mult(delay, t_id):
    global val2
    while True:
        time.sleep(delay)
        val2 = val2 * 2
_thread.start_new_thread(th_func_add, (2,1))
_thread.start_new_thread(th_func_mult, (3,2))

while True:
    time.sleep(1)
    print(val1)
    print(val2)