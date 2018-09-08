# Pythonic:
# An idea or piece of code which closely follows the most common idioms
# of the Python language, rather than implementing code using concepts
# common to other languages. For example, a common idiom in Python is
# to loop over all elements of an iterable using a for statement.
# Many other languages don’t have this type of construct, so people
# unfamiliar with Python sometimes use a numerical counter instead:
# for i in range(len(food)):
#     print(food[i])
# As opposed to the cleaner, Pythonic method:
# for piece in food:
#    print(piece)

# namespace:
# The place where a variable is stored. Namespaces are implemented as
# dictionaries. There are the local, global and built-in namespaces
# as well as nested namespaces in objects (in methods). Namespaces
# support modularity by preventing naming conflicts. For instance, the
# functions builtins.open and os.open() are distinguished by their
# namespaces. Namespaces also aid readability and maintainability by
# making it clear which module implements a function. For instance,
# writing random.seed() or itertools.islice() makes it clear that those
# functions are implemented by the random and itertools modules, respectively.

'''
>>>import this
The Zen of Python, by Tim Peters

Beautiful is better than ugly.
Explicit is better than implicit.
Simple is better than complex.
Complex is better than complicated.
Flat is better than nested.
Sparse is better than dense.
Readability counts.
Special cases aren't special enough to break the rules.
Although practicality beats purity.
Errors should never pass silently.
Unless explicitly silenced.
In the face of ambiguity, refuse the temptation to guess.
There should be one-- and preferably only one --obvious way to do it.
Although that way may not be obvious at first unless you're Dutch.
Now is better than never.
Although never is often better than *right* now.
If the implementation is hard to explain, it's a bad idea.
If the implementation is easy to explain, it may be a good idea.
Namespaces are one honking great idea -- let's do more of those!
'''

import module # An object that serves as an organizational unit of Python
              # code. Modules have a namespace containing arbitrary
              # Python objects. Modules are loaded into Python by the
              # process of importing.

class Class: # Any data with state (attributes or value) and defined
             # behavior (methods). Also the ultimate base class of
             # any new-style class.
    def __init__(self,argument):
        self.name = argument
    def method(): # A function which is defined inside a class body.
                  # If called as an attribute of an instance of that class,
                  # the method will get the instance object as its first
                  # argument (which is usually called self).
                  # See function and nested scope.
        print("output of a method")
        
object = Class("a name defined by a class argument")
print(object.name)
object.method()