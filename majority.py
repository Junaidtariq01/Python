def majority_element(nums):
    element = None
    count = 0
    for i in nums:
        if count == 0:
            element = i
        if i == element:
            count = count+1
        else:
            -1


    return element


arr = [2,2,2,1,1,1,2,2,2,1,1]
print(majority_element(arr))  