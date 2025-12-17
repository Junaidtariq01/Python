def quicksort(arr):
    if len(arr) <= 1:
        return arr
    else:
        pivot = arr[len(arr)//2]

        for x in arr:
            if x < pivot:
                left = x 
        for x in arr:
             if x == pivot:
                 middle = x
        for x in arr:
             if x > pivot:
                 right = x
        return quicksort(left)+middle+quicksort(right)



arr = []
n = int(input("Enter the size of ist array "))

for i in range(n):
    element = int(input(f"Enter {i+1} element : "))
    arr.append(element)

print(arr)

sortedArr = quicksort
print("arrau in quick",sortedArr)pp