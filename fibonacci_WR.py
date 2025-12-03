n = int(input("Enter any nth number:"))
a=0
b=1
fab=0
for i in range(n):
    fab=b+a
    print(fab)
    a=b
    b=fab

print("Nth element of fab :", fab)


