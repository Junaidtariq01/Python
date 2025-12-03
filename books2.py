books = {
    'python' : True,
    'chemistry': True,
    'coa':True,
    'maths':True
}
issued_books = {}

def view_books():
    print('available books')
    for book, available in books.items():
        if available:
            print("-",book)
def issue_book():
    book_name= input("Enter the book for issue ")
    if book_name in books:
        if books[book_name]:
            days = int(input("Enter how many days ?"))
            issued_books[book_name]=days
            books[book_name] = False
            print("issued")
        else:
            print("Sorry isssued!")
    else:
        print("book not found")

x = int(input("Enter your choice "))
if x== 1:
    view_books()
elif x==2:
    issue_book()
else:
    print("Thanks")