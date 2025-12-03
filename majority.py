def majority_element(nums):
    candidate = None
    count = 0

    for num in nums:
        if count == 0:
            candidate = num
        count += (1 if num == candidate else -1)

    return candidate

x=int(input("Enter the size of elements"))
for i in x:
   ele = x[i]




arr = [2, 2, 1, 1, 1, 2, 2]
print(majority_element(arr))   # Output: 2



