# x = int(input("Enter the no of books "))
books= ['physics','chemistry','maths','COA','python']

print("list of available books:")
for i in range(len(books)):
    print(books[i])

#issuing a book
issue = input("Enter the book you want purchase:")
print(issue,"book is purchased")
print("list of available books after purchase:")
#books = books-1
for i in range(len(books)):
    if(issue == books[i]):
        continue
    print(books[i])

#returing a book
ret = input("Enter the book you want to return:")
books[len(books)-1] = ret

print(ret,"book have been returned")

print("list of available books after return:")

for i in range(len(books)):
    print(books[i])

#fine after 15 days
