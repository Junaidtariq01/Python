#Stock prices
arr = [45,435,3400,23,54,5650]
# j=len(arr)-1
max_profit=0

for i in range(len(arr)):
    if arr[i] > max_profit:
        max_profit=arr[i]
        
print("Max profit=",max_profit)