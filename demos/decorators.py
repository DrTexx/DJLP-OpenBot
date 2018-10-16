'''def get_text(name):
    return("lorem ipsum, {} dolor sit amet".format(name))

def p_decorate(func):
    def func_wrapper(name):
        return("<p>{}</p>".format(func(name)))
    return(func_wrapper)

my_get_text = p_decorate(get_text)

print(my_get_text("John"))
'''

'''
def do_twice(func):
    def wrapper_do_twice():
        func()
        func()
    return wrapper_do_twice

@do_twice
def say_whee():
    print("Whee!")
    
say_whee()
'''

# -- ORIGINAL -- #
import time
def calc_square(numbers):
    start = time.time()
    result = []
    for number in numbers:
        result.append(number*number)
    end = time.time()
    print("calc_square took {} mil seconds".format((end-start)*1000))
    return(result)

calc_square(range(100000))

# -- WITH WRAPPERS -- #
# functions are first-class objects in Python
# hold control + 1 when a spelling mistake is present to add it to dictionary
def time_it(func): 
    def wrapper(*args, **kwargs):
        start = time.time()
        result = func(*arg,**kwargs)
        end = time.time()
        print("{} took {} mil seconds".format(func.__name__,(end-start)*1000))
        return(result)
