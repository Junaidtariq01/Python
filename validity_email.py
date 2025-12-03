import validators
x = (input("Enter the email address"))
if validators.email(x):
    print("Valid email")
else:
    print("invalid email")

