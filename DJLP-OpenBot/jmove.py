'''
Created on 20 Sep. 2018

@author: Denver
'''


def is_joint(arg):
    if type(arg) is str: return(True)
    else: return(False)
def angle_type_test(arg):
    if (type(arg) is int) or (type(arg) is float): return(True)
    else: return(False)
def joint_on_robot(arg,rob_obj):
    if arg in rob_obj.joints: return(True)
    else: return(False)
def joint_is_active(arg,rob_obj):
    return(rob_obj.joints[arg].isactive)
def gen_sublist_n_items(my_list,n):
    # credit to https://stackoverflow.com/a/15890829
    return([my_list[x:x+n] for x in range(0, len(my_list),n)])

''' MAIN PART OF JAI '''
def jmove_interpreter(rob_obj,*args):
    arg_pairs = gen_sublist_n_items(args, 2) # split every two items into sublists
    output_pairs = []
    for pair in arg_pairs: # for each sublist
        print(pair)
        if len(pair) is 2:
            print("pair contains 2 items")
            j = pair[0] # joint's name
            ang = pair[1] # joint's requested angle
            if is_joint(j) and angle_type_test(ang): # if the first pair item is a joint
                print("pair items are valid")
                if joint_on_robot(j,rob_obj): # check if joint on robot
                    print("the joint is on the robot")
                    if joint_is_active(j,rob_obj): # check if joint is active on robot
                        print("the joint is active")
                        print("relying on movement to check if angle out of bounds")
                        output_pairs.append({'joint': rob_obj.joints[j], 'angle': ang})
                        '''PASS ONTO JOINT MANAGER'''
                else: print("pair[0]:[joint {} isn't on robot:[{}]".format(j,rob_obj),end=" ")
            else: raise TypeError("input must be: [axis name] [new angle]")
        else: raise TypeError("input must be: [axis name] [new angle]")
    return(output_pairs)
        
def jmove(rob_obj,*args):
    
    j_targets = jmove_interpreter(rob_obj,*args)
    for j_tar in j_targets:
        print("joint name:[{}] angle:[{}]".format(j_tar['joint'].name,j_tar['angle']))
    print("BREAKING LOOP, REMOVE THIS LATER")
    do_it = False
    if do_it is True:
        rob_obj.manager.targets(j_targets)
        print("jmove_set!")

if __name__ == "__main__":
    print("running as main")
