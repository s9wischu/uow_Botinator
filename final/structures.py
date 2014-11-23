import xml.etree.ElementTree as xml
import lxml.html as html


class Amount: 
    """ An amount in dollars """
    def __init__ (self, dollars, cents):
        self.dollars = int(dollars)
        self.cents = int(cents)

    def fromCents(self, cents):
        self.cents = int(cents) % 100
        self.dollars = int(cents) / 100

    def toCents(self): 
        return self.dollars * 100 + self.cents

    def getCentPart(self): 
        return self.cents

    def getDollarPart(self): 
        return self.dollars

    def add (self, other): 
        self.fromCents(self.toCents() + other.toCents())

    def multiply (self, factor):
        self.fromCents(int(self.toCents() * factor))
        
    def __str__(self): 
        return "$" + str(self.dollars) + "." + ("%02d" % self.cents)

    def writeXML(self, parent): 
        node = xml.SubElement(parent, "amount")
        nodeDollars = xml.SubElement(node, "dollars")
        nodeDollars.text = str(self.getDollarPart())
        nodeCents = xml.SubElement(node, "cents")
        nodeCents.text = str(self.getCentPart())

    @staticmethod
    def fromXML(node): 
        amount = Amount(0, 0) 
        subnode = node.find("amount")
        amount.cents = int(float(subnode.find("cents").text))
        amount.dollars = int(float(subnode.find("dollars").text))

        return amount

class Weight: 
    """ Weight in ounces """
    def __init__(self, weight): 
        self.weight = float(weight)

    def getWeight(self): 
        return self.weight

    def setWeight(self, weight): 
        self.weight = float(weight)

    def add(self, other): 
        self.weight += other.weight

    def __str__(self):
        return str(self.weight) + "oz"

    def writeXML(self, parent): 
        node = xml.SubElement(parent, "weight")
        node.text = str(self.getWeight())
    
    @staticmethod
    def fromXML(node): 
        weight = Weight(0.0)
        weight.setWeight(float(node.find("weight").text))
        return weight

class Item:
    """ An item in the shop """

    def __init__(self):
        self.amount = Amount(0, 0)
        self.name = "null"
        self.id = -1
        self.weight = Weight(0.0)
        self.fiducialNum = -1
        self.description = ""
        self.imagePath ="../img/none.jpg"
        self.contains = []

    # SETTERS
    def setAmount(self, amount): 
        self.amount = amount

    def setName(self, name): 
        self.name = str(name)

    def setId(self, id): 
        self.id = int(id)

    def setWeight(self, weight): 
        self.weight = weight

    def setFiducialNum(self, fiducialNum): 
        self.fiducialNum = int(fiducialNum)

    def setDescription(self, description): 
        self.description = str(description)

    def setImagePath(self, path):
        self.imagePath = str(path)
        
    def addContains(self, content): 
        self.contains.append(content)

    # GETTERS
    def getAmount(self):
        return self.amount

    def getName(self): 
        return self.name

    def getId(self): 
        return self.id

    def getWeight(self):
        return self.weight

    def getFiducialNum(self):
        return self.fiducialNum

    def getDescription(self): 
        return self.description

    def getImagePath(self):
        return self.imagePath
        
    def getContains(self): 
        import copy
        return list(copy.deepcopy(self.contains))

    def __str__(self):
        result = "[Item id=" 
        result += str(self.getId()) 
        result += ", name=\"" 
        result += str(self.getName()) 
        result += "\", price=" 
        result += str(self.getAmount())
        result += ", fiducial="
        result += str(self.getFiducialNum())
        result += ", weight="
        result += str(self.getWeight())
        result += ", image="
        result += str(self.getImagePath())
        result += "]"
        return result

    def writeXML(self, parent): 
        node = xml.SubElement(parent, "item")
        nodeId = xml.SubElement(node, "id")
        nodeId.text = str(self.getId())
        nodeName = xml.SubElement(node, "name")
        nodeName.text = str(self.getName())
        nodeAmount = xml.SubElement(node, "amount")
        self.getAmount().writeXML(nodeAmount)
        nodeFiducial = xml.SubElement(node, "fiducialNum")
        nodeFiducial.text = str(self.getFiducialNum())
        nodeWeight = xml.SubElement(node, "weight")
        self.getWeight().writeXML(nodeWeight)
        nodeDescription = xml.SubElement(node, "description")
        nodeDescription.text = self.getDescription()
        nodeImagePath = xml.SubElement(node, "imagePath")
        nodeImagePath.text = self.getImagePath()
        for contains in self.getContains():
            nodeContains = xml.SubElement(node, "contains")
            nodeContains.text = contains

    @staticmethod
    def fromXML(node): 
        item = Item()
        item.setId(int(float(node.find("id").text)))
        item.setName(node.find("name").text)
        item.setAmount(Amount.fromXML(node.find("amount")))
        item.setFiducialNum(int(float(node.find("fiducialNum").text)))
        item.setWeight(Weight.fromXML(node.find("weight")))
        item.setDescription(node.find("description").text)
        item.setImagePath(node.find("imagePath").text)
        for node in node.iter("contains"):
            item.addContains(node.text)
        return item

    def getHTMLListString(self):
        """ Returns html for list item """
        from lxml.etree import ElementTree
        from xml.dom import minidom
        root = html.parse("html/product_list_item.html").getroot()
        root.get_element_by_id("field_image").attrib["src"] = self.getImagePath()
        root.get_element_by_id("field_price").text = str(self.getAmount())
        root.get_element_by_id("field_name").text = self.getName()
        root.get_element_by_id("top").attrib["item_id"] = str(self.getId())
        root.get_element_by_id("top").attrib["fiducial"] = str(self.getFiducialNum())
        tree = ElementTree(root)
        return html.tostring(tree, encoding="utf-8")
        
    def getStructure(self): 
        structure = {}
        structure["image_path"] = self.getImagePath(); 
        structure["amount"] = str(self.getAmount())
        structure["name"] = self.getName()
        structure["description"] = self.getDescription() 
        structure["id"] = str(self.getId())
        structure["fiducial"] = str(self.getFiducialNum())
        if len(self.getContains()) > 0: 
            structure["ingredients"] = reduce(lambda a, b : str(a) + ", " + str(b), self.getContains())
        else: 
            structure["ingredients"] = ""
        return structure

class Customer: 
    """ A customer """
    
    def __init__(self):
        self.id = -1
        self.name = "null"
        self.fiducialNum = -1
        self.allergies = []
        self.accountBalance = Amount(0, 0)

    # SETTERS
    def setName(self, name): 
        self.name = str(name)

    def setId(self, id): 
        self.id = int(id)

    def setFiducialNum(self, fiducialNum): 
        self.fiducialNum = int(fiducialNum)

    def addAllergy(self, name):
        self.allergies.append(str(name))

    def setAccountBalance(self, balance): 
        self.accountBalance = balance

    # GETTERS

    def getName(self): 
        return self.name

    def getFiducialNum(self):
        return self.fiducialNum

    def getId(self): 
        return self.id
    
    def getAllergies(self): 
        import copy
        return list(copy.deepcopy(self.allergies))

    def getAccountBalance(self): 
        return self.accountBalance
        
    def getStructure(self): 
        structure = {}
        structure["name"] = self.getName()
        structure["fiducial"] = str(self.getFiducialNum())
        structure["id"] = str(self.getId())
        structure["allergies"] = self.getAllergies()
        structure["balance"] = str(self.getAccountBalance())
        return structure
        
    def isAllergic(self, item): 
        """ Finds out if this customer has an allergy to the product passed
            as parameter. Returns [] if no allergy, or a list of the problematic
            ingredients. """
        result = []
        for allergy in self.getAllergies(): 
            if allergy.lower() in map(lambda x : x.lower(), item.getContains()): 
                result.append(allergy)
        return result
        

    def __str__(self):
        result = "[Customer id="
        result += str(self.getId())
        result += ", name=\""
        result += str(self.getName())
        result += "\", fiducialNum="
        result += str(self.getFiducialNum())
        result += ", allergies="
        result += str(self.getAllergies())
        result += ", balance="
        result += str(self.getAccountBalance())
        result += "]"
        return result

    def writeXML(self, parent): 
        """ Writes xml node for customer. """
        node = xml.SubElement(parent, "customer")
        nodeName = xml.SubElement(node, "name")
        nodeName.text = str(self.getName())
        nodeFiducial = xml.SubElement(node, "fiducialNum")
        nodeFiducial.text = str(self.getFiducialNum())
        nodeId = xml.SubElement(node, "id")
        nodeId.text = str(self.getId())
        nodeBalance = xml.SubElement(node, "balance")
        self.getAccountBalance().writeXML(nodeBalance)
        nodeAllergies = xml.SubElement(node, "allergies")
        for allergy in self.getAllergies(): 
            nodeAllergy = xml.SubElement(nodeAllergies, "allergy")
            nodeAllergy.text = allergy

    @staticmethod
    def fromXML(node): 
        """ Loads customer from xml """
        customer = Customer()

        customer.setName(node.find("name").text)
        customer.setFiducialNum(int(float(node.find("fiducialNum").text)))
        customer.setId(int(float(node.find("id").text)))
        customer.setAccountBalance(Amount.fromXML(node.find("balance")))
        
        for allergy in node.find("allergies").iter("allergy"):
            customer.addAllergy(allergy.text)

        return customer
        
        
class Database: 
    """ Stores all items of the store and all customers. 
        Supports storing to / reading from xml database. """
    def __init__(self): 
        self.customers = []
        self.items = []

    def addCustomer(self, customer): 
        self.customers.append(customer)

    def addItem(self, item): 
        self.items.append(item)

    def getCustomers(self): 
        import copy
        return list(copy.deepcopy(self.customers))

    def getItems(self): 
        import copy
        return list(copy.deepcopy(self.items))
    
    def getXML(self): 
        """ Returns the xml string of the database """
        node = xml.Element("database")
        for customer in self.getCustomers():
            customer.writeXML(node)
        
        for item in self.getItems(): 
            item.writeXML(node)
        return node
        
    def getItemByFiducialNumber(self, num): 
        """ Returns the item with the given fiducial number. Returns None
            if there is no item with the given fiducial number """
        for item in self.getItems(): 
            if item.getFiducialNum() == num: 
                return item
                
    def getCustomerByFiducialNumber(self, num): 
        """ Returns the customer with the given fiducial number. Returns None
            if there is no customer with the given fiducial number """
        for customer in self.getCustomers(): 
            if customer.getFiducialNum() == num: 
                return customer

    def __str__(self):
        result = "DATABASE\n"
        result += "Customers:\n"
        for customer in self.getCustomers(): 
            result += "  " + str(customer) + "\n"

        result += "Items:\n"
        for item in self.getItems(): 
            result += " " + str(item) + "\n"

        return result
        
    def search(self, searchString): 
        """ Returns all items that have 'searchString' in their name or
            in their description or in their contents"""
        result = []
        
        searchString = searchString.lower()
        searchString = searchString.replace("+", " ")
        print searchString
        for item in self.getItems(): 
            if (item.getName().lower().find(searchString) >= 0 or
                item.getDescription().lower().find(searchString) >= 0):
                result.append(item)
                continue
            
            for cont in item.getContains(): 
                if cont.lower().find(searchString) >= 0:
                    result.append(item)
                    break
                
        return result

    @staticmethod
    def fromXML(node): 
        """ Returns a database from the root node of an xml tree """
        database = Database()
        for customer in node.iter("customer"):
            database.addCustomer(Customer.fromXML(customer))

        for item in node.iter("item"):
            database.addItem(Item.fromXML(item))

        return database

    @staticmethod
    def fromXMLFile(filename): 
        """ Returns the database store at the location specified by 'filename'. """
        tree = xml.parse(filename)
        return Database.fromXML(tree.getroot())

    def toFile(self, filename):
        """ Stores this database to the location specified by 'filename'. """
        from xml.etree.ElementTree import ElementTree
        root = self.getXML()
        tree = ElementTree(root)
        tree.write(filename, encoding="utf-8")

class ItemList: 
    """ List of item that a customer has currently in shopping cart """
    
    # Id of next item
    currentId = 100
    
    def __init__(self): 
        self.list = []
        
    def addItem(self, item): 
        """ Add item to list """
        item.setId(ItemList.currentId)
        ItemList.currentId += 1
        self.list.append(item)
        
    def getItems(self):
        """ Get a copy of all items in the list """
        import copy
        return list(copy.deepcopy(self.list))
        
    def getTotalAmount(self):
        """ Returns the total amount of the items in the list """
        total = Amount(0, 0)
        for item in self.getItems():
            total.add(item.getAmount())
        return total
        
    def getById(self, id): 
        """ Returns the first item in the list with the given id (or none if 
            there is no such item) """
        for item in self.list: 
            if item.getId() == id:
                return item
                
    def removeById(self, id):   
        """ Removes the first item with the given id (if any). """
        for i in range(len(self.list)):
            if self.list[i].getId() == id: 
                del self.list[i]
                return
        
                
        

    
