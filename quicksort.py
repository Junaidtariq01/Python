def quicksort(arr):
    if len(arr) <= 1:
        return arr

    pivot = arr[len(arr) // 2]   #middle element as pivot
    left = [x for x in arr if x < pivot]
    middle = [x for x in arr if x == pivot]
    right = [x for x in arr if x > pivot]

    return quicksort(left) + middle + quicksort(right)


numbers = [10, 7, 8, 9, 1, 5,878]
sorted_numbers = quicksort(numbers)
print(sorted_numbers)
