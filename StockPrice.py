#Stock prices
# arr = []
# n = int(input("Enter many elements? "))

# for i in range(n):
#     element = int(input(f"Enter element {i+1}: "))
#     arr.append(element)

arr = [45,435,3400,23,54,565]
# j=len(arr)-1
max_profit=0

for i in range(len(arr)):
    if arr[i] > max_profit:
        max_profit=arr[i]

print("Max profit=",max_profit)
