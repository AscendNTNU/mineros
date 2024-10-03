import json
import os

# Import json data
fileNames = ['blocks', 'items', 'recipes']
dictionaries = []
filePaths = [os.path.join(os.path.dirname(__file__), f'{fileName}.json') for fileName in fileNames]

blockIDDic = {}
itemIDDic  = {}
recipeIDDic = {}

with open(filePaths[0], 'r') as file:
    jsonFile = json.load(file) 
    for item in jsonFile:
        blockIDDic[item['name']] = item['id']

with open(filePaths[1], 'r') as file:
    jsonFile = json.load(file) 
    for item in jsonFile:
        itemIDDic[item['name']] = item['id']

with open(filePaths[2], 'r') as file:
    jsonFile = json.load(file) 
    for item in jsonFile:
        block_name = ''
        for name, ID in itemIDDic.items():
            if ID == int(item):
                block_name = name
        recipeIDDic[block_name] = jsonFile[item]
