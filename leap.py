n = int(input("Enter any year: "))
if(n%4==0 and 0!=n%400):
    print("Current year",n,"is a leap year ")
else:
    print("not a leap year")
