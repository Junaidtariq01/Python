n = int(input("Enter ist number: "))
m = int(input("Enter 2nd number: "))

r = int(input("Enter 1,2,3,4 for +,-,*,/ resp "))

if(r==1):
    print("Addition:", n+m)
elif(r==2):
    print("Subtraction:",n-m)
elif(r==3):
    print("Multiple:",n*m)
elif(r==4 and n>m):
    print("Division:",n/m)
else:
    print("Error found recheck")