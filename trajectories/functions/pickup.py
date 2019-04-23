
# coding: utf-8

# In[1]:


# if i is in bound return a proper list otherwise return false
def cheese_pickup_from_stack(i, world, is_init):
    goingInPointNum = 100
    bringingUpPointNum = 20
    goingInLength = 0.05
    bringingUpLength = 0.05
    if i < 0:
        return False
    if i >= (goingInPointNum + bringingUpPointNum):
        return False
    
    pose = world.topCheeseSpatulaPose[:]
    if i < goingInPointNum:
        pose[2] = world.topCheeseSpatulaPose[2] - (world.totalCheeseNum - world.curCheeseNum + 1)*world.cheeseWidth 
        pose[1] = world.topCheeseSpatulaPose[1] - goingInLength*float(goingInPointNum - i)/float(goingInPointNum)
    else: 
        j = i - goingInPointNum
        pose[2] = world.topCheeseSpatulaPose[2] - (world.totalCheeseNum - world.curCheeseNum + 1)*world.cheeseWidth                                                 + bringingUpLength*float(j)/float(bringingUpPointNum)
        pose[1] = world.topCheeseSpatulaPose[1]
        
    is_free_driving = False
    is_reaching = False
    gripper_force = 10
    gripper_loc = 100
    gripper_speed = 100
    
    point = pose + [is_free_driving, is_reaching, gripper_force, gripper_loc, gripper_speed]
    
    return point

def cheese_pickup_next_from_stack(i, world, is_init):
    if is_init == True and world.pickingJustStarted == False:
        world.cheesePicked()
        world.pickingJustStarted = True
    else:
        world.pickingJustStarted = False
        
    return cheese_pickup_from_stack(i, world, False)

def patty_pickup_from_stack(i, obj):
    return 2*i

