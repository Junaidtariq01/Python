# arr = []

# element = input("Enter an element: ")
# arr.append(element)

# print(arr)
arr = []

n = int(input("Enter elements "))

for i in range(n):
    element = input(f"Enter element {i+1}: ")
    arr.append(element)

print(arr)
