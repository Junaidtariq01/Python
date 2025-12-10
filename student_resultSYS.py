# x = (input("Enter the Name of student:"))
# student={'Name':"junaid",
#         "class":"3rd" , 
#         "RollNo": 123 ,
#         "Marks" : 98}

# class Student:
#     "name"= x

# p= Student()





class Person:
    def __init__(self, name, rollno, sem, marks):
        self.name = name
        self.rollno = rollno
        self.sem = sem
        self.marks = marks

p2 = Person(name="junaid")
print(p2.name)
print(p2.age)
print(p2.marks) 