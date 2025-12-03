def rotate_right(nums, k):
    n = len(nums)
    k %= n  # handle k > n
    return nums[-k:] + nums[:-k]


# Example
arr = [1, 2, 3, 4, 5, 6, 7]
k = 3
print(rotate_right(arr, k))   # Output: [5, 6, 7, 1, 2, 3, 4]
