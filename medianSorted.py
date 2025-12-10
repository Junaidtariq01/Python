#median of 2 sorted array
#shortest path algo
#quick sort
# reversed an array 

#array1
arr = []
n = int(input("Enter the size of ist array "))

for i in range(n):
    element = int(input(f"Enter {i+1} element : "))
    arr.append(element)

print(arr)

#array2
arr2 = []
m = int(input("Enter the size of 2nd array "))

for i in range(m):
    element2 = int(input(f"Enter {i+1} element : "))
    arr2.append(element2)

print(arr2)

# arr.sort()
# arr2.sort()

merged_arr = arr + arr2

merged_arr.sort()

print(merged_arr)

l = len(merged_arr)


if l%2 == 0:          #even
    mid1= merged_arr[l//2 -1]
    mid2= merged_arr[l//2]
    median = (mid1+mid2)/2
    print("median =",median)
else:                  #odd
    median = merged_arr[l//2]
    print("median =",median)