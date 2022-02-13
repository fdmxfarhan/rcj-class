class Person:
    def __init__(self, name, age, height, weight):
        self.name = name
        self.age = age
        self.height = height
        self.weight = weight
    def sayHi(self):
        print(self.name + ' sayed Hi !!')
    
ali = Person('Alireza', 15, 170, 80)
mohammad = Person('Mohammad', 18, 160, 50)
