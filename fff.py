# def add(x, y=2):
#     return x + y

# print(add(3))
# list = [1,3,4,5,6,"hthr",3]
# tup = (12,23,22,44,"the price",22)

# print(tup[5])
# print(list[6])
# list.pop()
# print(list[5])

info = {
    "name" : "junaid",
    "cgpa" : 9.9,
    "list" : [9,8,5],
    "int" : 4,
    "float" :5.555
}
# print(info)

# print(info["float"])
# set = {1,4,5,6,7}
# print(set)

# x=1.0
# y=1.0
# z = x|y
# print(z)
# x = [1,2,3,4]

# x = [1, 2, 3]
# y = x
# y.append(4)
# print(x)


# a = [1, 2, 3]
# b = a[:]
# b.append(4)
# print(b)

# for i in range(1, 4):
#     print(i * " *")

# x = [0, 1, 2, 3]
# print(x[-1:-3:-1])

# x = [1, 2, 3]
# print(x * 2)

# a = [1, 2, 3];
# b = [1, 2, 3];
# print(a is b, a == b),

# def print_info(info):
#     for key, value in info.items():
#         print(f"{key}: {value}")


# print_info(info)
# print_info(name="Alice", age=30, city="New York")
# print_info(product="Laptop", price=1200, brand="Dell", warranty="1 year")

#lets know my age in seconds
name = input("Enter your name :")
years = int(input("Enter your current age 'in years':"))
age_sec = years*12*30*24*60
print(name,"s age in sec",age_sec)