from networktables import NetworkTables
from collections import namedtuple
import time
import logging

logging.basicConfig(level=logging.DEBUG)

NetworkTables.initialize(server='10.49.3.2')
table = NetworkTables.getTable("dataTest")
y = 0

while True:
    print(table.getNumber('X', 0))
    table.putNumber("Y", y)
    y += 1.027
    time.sleep(0.1)