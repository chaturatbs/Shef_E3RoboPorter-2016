def answer(data, n):
    # your code here
    testInt = min(data)
    intCount = 0
    maxList = max(data)
    backupList = []
    reLoop = False
    while testInt < maxList:
        for element in data:
            if element == testInt:
                intCount += 1
            if intCount > n:
                for copyVal in data:
                    if copyVal != testInt:
                        backupList.append(copyVal)
                        reLoop = True
                data = backupList
                backupList = []

            if reLoop:
                reLoop = False
                break

        testInt += 1
        intCount = 0

    return data

print (answer([5,5,15,10,7],1))