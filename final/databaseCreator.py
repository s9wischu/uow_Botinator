#!/usr/bin/env python

from structures import *

if __name__ == '__main__':

    database = Database()

    customer = Customer()
    customer.addAllergy("Peanuts")
    customer.setName("Michel Keller")
    customer.setAccountBalance(Amount(29, 99))
    customer.setId(20022)
    customer.setFiducialNum(6)
    database.addCustomer(customer)
    
    customer = Customer()
    customer.addAllergy("Milk")
    customer.setName("Winfried Schuffert")
    customer.setAccountBalance(Amount(99, 99))
    customer.setId(20023)
    customer.setFiducialNum(7)
    database.addCustomer(customer)
    
    customer = Customer()
    customer.addAllergy("Peanuts")
    customer.addAllergy("Gluten")
    customer.setName("Seth Vanderwilt")
    customer.setAccountBalance(Amount(49, 99))
    customer.setId(23984)
    customer.setFiducialNum(8)
    database.addCustomer(customer)

    item = Item()
    item.setName("Peanut Butter")
    item.setAmount(Amount(5, 30))
    item.setFiducialNum(29484849)
    item.setWeight(Weight(12.4))
    item.setDescription("The best peanut butter around!")
    item.setImagePath("../img/peanutbutter.jpg")
    item.addContains("peanuts")
    database.addItem(item)
    
    item = Item()
    item.setName("Oreos")
    item.setAmount(Amount(2, 99))
    item.setFiducialNum(9323298324)
    item.setWeight(Weight(15.5))
    item.setDescription("Black and white cookies!")
    item.setImagePath("../img/oreos.jpg")
    item.addContains("milk")
    item.addContains("gluten")
    database.addItem(item)
    
    item = Item()
    item.setName("Trader Joe's Organic Peppermints")
    item.setAmount(Amount(1, 99))
    item.setFiducialNum(23993)
    item.setWeight(Weight(1.41))
    item.setDescription("USDA organic")
    item.setImagePath("../img/peppermints.jpg")
    database.addItem(item)
    
    item = Item()
    item.setName("Peanuts (Salted)")
    item.setAmount(Amount(2, 49))
    item.setFiducialNum(2394948)
    item.setWeight(Weight(12.00))
    item.setDescription("7 vitamins and minerals")
    item.setImagePath("../img/peanuts.jpg")
    item.addContains("peanuts")
    database.addItem(item)
    
    
    

    
    database.toFile("database.xml")
