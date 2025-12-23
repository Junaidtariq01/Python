def can_partition(nums):
    total = sum(nums)
    
    if total % 2 != 0:
        return False

    target = total // 2
    possible = set([0])

    for num in nums:
        new_sums = set()
        for s in possible:
            if s + num == target:
                return True
            new_sums.add(s + num)
        possible |= new_sums

    return target in possible

# Example
print(can_partition([1, 5, 11, 5]))  
