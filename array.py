def right(nums, k):
    n = len(nums)
    k %= n   
    return nums[-k:] + nums[:-k]

arr = [1, 2, 3, 4, 5, 6, 7]
k = 3
print(right(arr, k)) 
