#ifndef PULSE_DATA_H
#define PULSE_DATA_H
#include <stdio.h>
#include <stdint.h>
//#include "main.h"

//#define MAX_LENGTH 2500

extern uint16_t pulse_data1[MAX_LENGTH];

uint16_t pulse_data1[MAX_LENGTH] =
{
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
1,
2,
1,
-2,
-6,
-5,
2,
12,
14,
2,
-18,
-31,
-16,
23,
59,
50,
-19,
-111,
-134,
4,
4,
339,
810,
1285,
1628,
1772,
1740,
1622,
1511,
1456,
1450,
1462,
1464,
1452,
1438,
1432,
1435,
1440,
1439,
1434,
1430,
1433,
1446,
1464,
1483,
1499,
1512,
1524,
1536,
1548,
1558,
1564,
1565,
1563,
1562,
1563,
1568,
1573,
1576,
1576,
1574,
1571,
1569,
1569,
1570,
1570,
1568,
1566,
1564,
1565,
1567,
1571,
1575,
1581,
1586,
1591,
1593,
1588,
1579,
1565,
1552,
1542,
1536,
1534,
1536,
1539,
1544,
1549,
1553,
1554,
1553,
1549,
1547,
1546,
1548,
1552,
1555,
1557,
1558,
1558,
1558,
1560,
1563,
1565,
1565,
1563,
1558,
1553,
1549,
1549,
1552,
1559,
1568,
1579,
1589,
1595,
1595,
1588,
1576,
1563,
1552,
1546,
1545,
1547,
1549,
1550,
1551,
1551,
1553,
1556,
1557,
1557,
1554,
1550,
1547,
1547,
1548,
1549,
1551,
1551,
1550,
1550,
1552,
1556,
1562,
1566,
1568,
1567,
1564,
1563,
1566,
1572,
1582,
1591,
1597,
1596,
1587,
1574,
1559,
1546,
1538,
1534,
1534,
1536,
1540,
1544,
1547,
1548,
1548,
1547,
1545,
1545,
1546,
1549,
1553,
1555,
1555,
1554,
1554,
1558,
1564,
1572,
1578,
1578,
1573,
1565,
1559,
1558,
1563,
1573,
1585,
1593,
1595,
1590,
1578,
1563,
1546,
1533,
1523,
1520,
1521,
1527,
1534,
1542,
1547,
1549,
1548,
1545,
1544,
1545,
1548,
1551,
1551,
1550,
1548,
1547,
1549,
1552,
1556,
1558,
1559,
1557,
1554,
1550,
1547,
1547,
1555,
1570,
1591,
1609,
1619,
1614,
1596,
1570,
1544,
1524,
1512,
1509,
1512,
1520,
1532,
1545,
1558,
1567,
1569,
1565,
1558,
1551,
1547,
1547,
1549,
1550,
1551,
1552,
1555,
1561,
1566,
1567,
1564,
1556,
1549,
1547,
1553,
1565,
1579,
1590,
1595,
1594,
1588,
1579,
1568,
1556,
1541,
1528,
1520,
1519,
1526,
1540,
1554,
1565,
1566,
1559,
1545,
1532,
1525,
1527,
1535,
1544,
1550,
1549,
1543,
1536,
1533,
1536,
1542,
1550,
1555,
1559,
1563,
1573,
1590,
1611,
1631,
1642,
1639,
1623,
1597,
1568,
1540,
1518,
1505,
1500,
1503,
1513,
1525,
1536,
1543,
1544,
1542,
1539,
1538,
1540,
1543,
1545,
1546,
1547,
1548,
1551,
1554,
1556,
1556,
1555,
1553,
1554,
1557,
1565,
1579,
1600,
1628,
1658,
1683,
1694,
1685,
1657,
1613,
1564,
1519,
1484,
1464,
1456,
1459,
1470,
1485,
1499,
1511,
1517,
1519,
1519,
1519,
1522,
1528,
1536,
1545,
1554,
1561,
1563,
1560,
1553,
1547,
1546,
1550,
1558,
1566,
1569,
1566,
1560,
1555,
1557,
1571,
1596,
1629,
1663,
1693,
1714,
1720,
1710,
1682,
1638,
1583,
1525,
1475,
1441,
1424,
1421,
1428,
1437,
1447,
1458,
1471,
1485,
1497,
1505,
1509,
1512,
1519,
1529,
1541,
1551,
1555,
1556,
1556,
1559,
1566,
1571,
1573,
1571,
1568,
1566,
1566,
1567,
1568,
1571,
1580,
1602,
1636,
1677,
1716,
1740,
1743,
1722,
1682,
1630,
1575,
1525,
1484,
1453,
1433,
1421,
1415,
1416,
1423,
1435,
1452,
1470,
1488,
1501,
1511,
1518,
1524,
1528,
1530,
1532,
1532,
1534,
1537,
1545,
1555,
1566,
1576,
1583,
1585,
1584,
1581,
1577,
1576,
1580,
1594,
1617,
1646,
1675,
1693,
1696,
1681,
1651,
1611,
1564,
1517,
1475,
1444,
1429,
1432,
1448,
1470,
1488,
1496,
1494,
1488,
1484,
1487,
1498,
1515,
1531,
1544,
1551,
1552,
1551,
1552,
1557,
1566,
1576,
1582,
1582,
1577,
1572,
1575,
1589,
1615,
1647,
1677,
1699,
1707,
1704,
1689,
1662,
1623,
1572,
1517,
1464,
1427,
1410,
1415,
1434,
1456,
1471,
1477,
1475,
1471,
1471,
1478,
1490,
1506,
1520,
1532,
1541,
1549,
1556,
1563,
1569,
1575,
1580,
1584,
1585,
1581,
1572,
1563,
1561,
1575,
1610,
1660,
1715,
1756,
1772,
1757,
1717,
1662,
1605,
1554,
1513,
1481,
1457,
1440,
1431,
1430,
1436,
1446,
1454,
1457,
1455,
1450,
1448,
1453,
1465,
1484,
1505,
1526,
1542,
1555,
1563,
1569,
1572,
1574,
1572,
1567,
1559,
1552,
1550,
1560,
1584,
1621,
1667,
1712,
1747,
1766,
1765,
1746,
1713,
1669,
1616,
1560,
1506,
1461,
1433,
1423,
1428,
1441,
1452,
1459,
1461,
1461,
1464,
1467,
1470,
1471,
1471,
1472,
1479,
1495,
1516,
1539,
1559,
1570,
1575,
1575,
1576,
1579,
1582,
1582,
1577,
1570,
1570,
1587,
1623,
1674,
1728,
1768,
1783,
1769,
1732,
1683,
1631,
1582,
1539,
1503,
1476,
1458,
1452,
1454,
1461,
1469,
1474,
1476,
1477,
1477,
1478,
1480,
1483,
1490,
1501,
1517,
1536,
1552,
1561,
1562,
1559,
1557,
1560,
1567,
1575,
1581,
1587,
1597,
1617,
1648,
1686,
1721,
1744,
1750,
1736,
1707,
1666,
1620,
1574,
1531,
1496,
1470,
1455,
1451,
1457,
1467,
1476,
1481,
1481,
1475,
1469,
1467,
1470,
1477,
1487,
1498,
1508,
1519,
1530,
1543,
1555,
1563,
1566,
1563,
1561,
1566,
1587,
1626,
1678,
1730,
1771,
1792,
1790,
1773,
1747,
1717,
1681,
1638,
1587,
1532,
1485,
1453,
1440,
1440,
1445,
1446,
1442,
1435,
1428,
1426,
1426,
1429,
1432,
1436,
1440,
1446,
1452,
1461,
1474,
1492,
1513,
1534,
1550,
1559,
1566,
1581,
1612,
1660,
1720,
1776,
1816,
1831,
1825,
1801,
1768,
1728,
1682,
1629,
1575,
1525,
1486,
1458,
1441,
1427,
1414,
1404,
1398,
1398,
1404,
1410,
1414,
1417,
1421,
1430,
1444,
1461,
1475,
1484,
1490,
1498,
1510,
1525,
1537,
1541,
1537,
1532,
1537,
1563,
1611,
1678,
1748,
1811,
1854,
1874,
1870,
1842,
1794,
1732,
1663,
1597,
1542,
1499,
1467,
1444,
1425,
1411,
1404,
1403,
1410,
1420,
1431,
1439,
1444,
1446,
1447,
1449,
1453,
1461,
1473,
1488,
1504,
1518,
1527,
1529,
1526,
1523,
1525,
1532,
1545,
1566,
1596,
1637,
1690,
1747,
1798,
1829,
1832,
1807,
1760,
1704,
1647,
1595,
1550,
1513,
1486,
1470,
1463,
1463,
1465,
1464,
1461,
1457,
1456,
1457,
1460,
1462,
1462,
1462,
1464,
1471,
1484,
1500,
1515,
1524,
1527,
1526,
1527,
1537,
1564,
1608,
1663,
1718,
1762,
1787,
1790,
1773,
1738,
1690,
1634,
1577,
1528,
1495,
1478,
1474,
1477,
1480,
1481,
1480,
1480,
1480,
1479,
1478,
1474,
1471,
1469,
1471,
1477,
1486,
1495,
1502,
1507,
1510,
1513,
1520,
1534,
1561,
1603,
1657,
1717,
1771,
1807,
1815,
1795,
1752,
1698,
1645,
1599,
1563,
1535,
1513,
1495,
1481,
1472,
1467,
1466,
1467,
1467,
1467,
1467,
1467,
1470,
1472,
1474,
1475,
1474,
1475,
1479,
1485,
1491,
1496,
1502,
1515,
1541,
1582,
1634,
1686,
1725,
1743,
1741,
1721,
1691,
1656,
1622,
1588,
1558,
1534,
1519,
1512,
1513,
1515,
1515,
1510,
1501,
1491,
1482,
1478,
1477,
1479,
1484,
1491,
1501,
1510,
1517,
1519,
1517,
1514,
1512,
1514,
1520,
1529,
1542,
1561,
1588,
1622,
1658,
1687,
1702,
1699,
1682,
1657,
1629,
1604,
1582,
1562,
1544,
1530,
1520,
1516,
1517,
1519,
1520,
1519,
1515,
1510,
1507,
1506,
1505,
1506,
1507,
1508,
1512,
1517,
1523,
1528,
1531,
1532,
1533,
1536,
1538,
1540,
1541,
1542,
1548,
1563,
1589,
1624,
1658,
1684,
1694,
1688,
1670,
1648,
1625,
1604,
1583,
1561,
1539,
1521,
1510,
1511,
1520,
1533,
1542,
1542,
1534,
1522,
1513,
1508,
1508,
1508,
1507,
1504,
1503,
1506,
1515,
1526,
1535,
1537,
1532,
1528,
1532,
1551,
1583,
1622,
1656,
1676,
1679,
1668,
1647,
1624,
1602,
1582,
1564,
1547,
1533,
1523,
1518,
1517,
1519,
1522,
1527,
1530,
1531,
1529,
1524,
1518,
1515,
1516,
1519,
1521,
1520,
1517,
1516,
1519,
1526,
1534,
1539,
1541,
1542,
1550,
1572,
1608,
1653,
1697,
1727,
1735,
1722,
1694,
1659,
1625,
1595,
1571,
1550,
1532,
1518,
1508,
1504,
1502,
1501,
1498,
1493,
1487,
1484,
1483,
1486,
1489,
1494,
1499,
1505,
1510,
1513,
1514,
1515,
1516,
1520,
1524,
1528,
1530,
1530,
1533,
1544,
1567,
1601,
1640,
1675,
1696,
1700,
1686,
1661,
1631,
1602,
1577,
1558,
1544,
1536,
1534,
1536,
1540,
1542,
1541,
1538,
1534,
1529,
1524,
1517,
1506,
1496,
1490,
1491,
1500,
1513,
1525,
1532,
1533,
1530,
1527,
1528,
1533,
1539,
1541,
1537,
1531,
1531,
1543,
1571,
1608,
1643,
1667,
1675,
1667,
1649,
1626,
1600,
1573,
1551,
1535,
1529,
1529,
1529,
1527,
1521,
1513,
1508,
1507,
1510,
1513,
1514,
1512,
1507,
1500,
1493,
1489,
1493,
1505,
1523,
1540,
1550,
1548,
1538,
1525,
1519,
1525,
1545,
1574,
1607,
1634,
1651,
1655,
1649,
1637,
1624,
1611,
1600,
1588,
1575,
1561,
1548,
1538,
1534,
1535,
1538,
1542,
1545,
1544,
1539,
1532,
1523,
1517,
1515,
1519,
1526,
1535,
1542,
1546,
1548,
1547,
1544,
1540,
1539,
1545,
1562,
1589,
1622,
1652,
1669,
1668,
1649,
1617,
1583,
1552,
1531,
1517,
1509,
1506,
1505,
1508,
1513,
1519,
1522,
1520,
1514,
1506,
1501,
1497,
1494,
1490,
1485,
1483,
1486,
1494,
1505,
1515,
1524,
1531,
1537,
1540,
1538,
1532,
1528,
1534,
1558,
1599,
1645,
1680,
1693,
1682,
1656,
1627,
1602,
1581,
1562,
1542,
1524,
1511,
1507,
1510,
1515,
1519,
1520,
1520,
1520,
1518,
1513,
1504,
1495,
1489,
1489,
1497,
1507,
1517,
1524,
1527,
1530,
1533,
1537,
1539,
1539,
1539,
1544,
1559,
1585,
1617,
1648,
1670,
1677,
1669,
1650,
1624,
1597,
1573,
1553,
1539,
1531,
1527,
1524,
1523,
1523,
1525,
1529,
1534,
1540,
1545,
1546,
1544,
1537,
1527,
1518,
1512,
1512,
1520,
1530,
1540,
1546,
1548,
1548,
1551,
1561,
1578,
1601,
1627,
1648,
1660,
1658,
1640,
1611,
1578,
1550,
1533,
1524,
1522,
1522,
1523,
1525,
1530,
1539,
1547,
1552,
1551,
1545,
1536,
1527,
1520,
1515,
1514,
1516,
1522,
1529,
1537,
1541,
1541,
1538,
1537,
1544,
1563,
1591,
1623,
1650,
1665,
1666,
1653,
1632,
1606,
1580,
1558,
1544,
1537,
1537,
1540,
1544,
1543,
1539,
1533,
1528,
1526,
1527,
1529,
1530,
1528,
1526,
1524,
1525,
1528,
1532,
1535,
1536,
1533,
1530,
1528,
1530,
1539,
1557,
1581,
1607,
1630,
1644,
1647,
1638,
1621,
1602,
1583,
1566,
1552,
1540,
1533,
1531,
1534,
1542,
1549,
1552,
1549,
1541,
1533,
1527,
1524,
1524,
1523,
1522,
1522,
1524,
1529,
1536,
1543,
1547,
1546,
1542,
1539,
1541,
1550,
1566,
1585,
1605,
1622,
1634,
1640,
1640,
1635,
1623,
1607,
1588,
1569,
1553,
1542,
1534,
1530,
1530,
1532,
1535,
1539,
1540,
1539,
1534,
1528,
1523,
1522,
1523,
1526,
1529,
1532,
1535,
1537,
1540,
1544,
1547,
1549,
1549,
1546,
1543,
1540,
1542,
1549,
1564,
1585,
1608,
1628,
1642,
1646,
1641,
1626,
1606,
1584,
1565,
1554,
1550,
1553,
1555,
1553,
1545,
1535,
1527,
1525,
1527,
1531,
1532,
1530,
1527,
1527,
1530,
1533,
1534,
1529,
1523,
1520,
1524,
1535,
1547,
1554,
1553,
1544,
1535,
1531,
1536,
1547,
1561,
1575,
1589,
1605,
1623,
1638,
1646,
1642,
1624,
1598,
1571,
1549,
1537,
1533,
1537,
1543,
1548,
1550,
1548,
1543,
1536,
1532,
1530,
1531,
1533,
1533,
1531,
1527,
1525,
1528,
1535,
1545,
1552,
1553,
1549,
1541,
1534,
1533,
1538,
1546,
1555,
1563,
1573,
1586,
1600,
1612,
1618,
1615,
1604,
1588,
1571,
1557,
1544,
1535,
1527,
1523,
1524,
1528,
1535,
1540,
1542,
1540,
1535,
1531,
1528,
1528,
1530,
1533,
1536,
1540,
1544,
1547,
1548,
1548,
1546,
1544,
1541,
1539,
1538,
1542,
1550,
1565,
1583,
1601,
1614,
1620,
1620,
1614,
1605,
1593,
1579,
1563,
1548,
1538,
1535,
1538,
1544,
1548,
1549,
1548,
1545,
1541,
1538,
1535,
1531,
1528,
1526,
1527,
1532,
1538,
1544,
1548,
1548,
1546,
1544,
1546,
1550,
1555,
1558,
1556,
1549,
1543,
1541,
1550,
1569,
1595,
1620,
1636,
1639,
1629,
1612,
1594,
1579,
1567,
1556,
1546,
1537,
1531,
1531,
1537,
1545,
1550,
1550,
1545,
1538,
1534,
1532,
1531,
1529,
1525,
1520,
1517,
1518,
1524,
1534,
1547,
1559,
1570,
1578,
1579,
1575,
1567,
1557,
1549,
1549,
1556,
1571,
1588,
1605,
1616,
1619,
1616,
1608,
1596,
1583,
1570,
1559,
1551,
1547,
1546,
1548,
1550,
1551,
1551,
1549,
1546,
1542,
1537,
1534,
1532,
1533,
1535,
1536,
1535,
1532,
1530,
1532,
1536,
1541,
1545,
1546,
1545,
1544,
1545,
1547,
1552,
1559,
1570,
1584,
1598,
1607,
1608,
1600,
1587,
1573,
1563,
1556,
1551,
1546,
1541,
1537,
1537,
1540,
1545,
1547,
1544,
1537,
1531,
1530,
1534,
1540,
1543,
1542,
1538,
1534,
1537,
1545,
1555,
1563,
1564,
1560,
1554,
1549,
1547,
1550,
1555,
1562,
1570,
1579,
1587,
1594,
1597,
1593,
1581,
1564,
1547,
1534,
1530,
1533,
1540,
1547,
1551,
1552,
1550,
1547,
1546,
1545,
1544,
1542,
1540,
1537,
1534,
1531,
1531,
1533,
1538,
1544,
1549,
1551,
1550,
1548,
1548,
1548,
1550,
1551,
1553,
1557,
1563,
1573,
1584,
1594,
1600,
1600,
1595,
1586,
1574,
1563,
1554,
1547,
1542,
1540,
1541,
1542,
1544,
1548,
1551,
1553,
1554,
1552,
1547,
1540,
1534,
1531,
1532,
1538,
1546,
1555,
1561,
1563,
1562,
1558,
1556,
1553,
1551,
1547,
1541,
1534,
1532,
1539,
1556,
1581,
1605,
1620,
1620,
1608,
1590,
1573,
1561,
1553,
1546,
1539,
1533,
1528,
1527,
1529,
1531,
1532,
1534,
1536,
1542,
1547,
1550,
1549,
1542,
1535,
1531,
1531,
1536,
1542,
1544,
1542,
1537,
1534,
1534,
1539,
1547,
1556,
1564,
1570,
1572,
1570,
1568,
1569,
1576,
1587,
1601,
1608,
1605,
1589,
1566,
1544,
1529,
1526,
1531,
1538,
1543,
1543,
1540,
1538,
1540,
1546,
1552,
1554,
1547,
1535,
1523,
1517,
1520,
1531,
1545,
1554,
1556,
1552,
1546,
1541,
1540,
1541,
1544,
1547,
1551,
1557,
1566,
1576,
1586,
1591,
1590,
1583,
1574,
1566,
1560,
1556,
1551,
1546,
1540,
1537,
1538,
1543,
1550,
1555,
1556,
1553,
1548,
1543,
1540,
1539,
1538,
1538,
1539,
1542,
1548,
1556,
1562,
1564,
1563,
1560,
1556,
1552,
1551,
1551,
1554,
1561,
1570,
1581,
1591,
1598,
1600,
1596,
1588,
1577,
1565,
1555,
1548,
1545,
1544,
1543,
1542,
1542,
1543,
1546,
1549,
1550,
1549,
1547,
1545,
1544,
1546,
1548,
1550,
1550,
1549,
1549,
1549,
1550,
1553,
1555,
1556,
1556,
1554,
1551,
1550,
1552,
1560,
1570,
1581,
1591,
1596,
1598,
1598,
1594,
1587,
1575,
1561,
1548,
1541,
1540,
1542,
1543,
1539,
1532,
1527,
1527,
1533,
1543,
1549,
1548,
1540,
1530,
1524,
1531,
1540,
1548,
1552,
1554,
1555,
1556,
1557,
1556,
1554,
1550,
1548,
1549,
1554,
1561,
1568,
1576,
1584,
1591,
1597,
1599,
1596,
1589,
1580,
1571,
1562,
1553,
1546,
1542,
1542,
1548,
1557,
1566,
1568,
1563,
1552,
1538,
1527,
1520,
1520,
1526,
1535,
1545,
1552,
1556,
1553,
1546,
1537,
1533,
1537,
1547,
1560,
1571,
1573,
1568,
1559,
1552,
1551,
1557,
1567,
1578,
1584,
1586,
1585,
1582,
1579,
1573,
1566,
1558,
1551,
1547,
1547,
1548,
1550,
1551,
1550,
1548,
1546,
1545,
1546,
1547,
1546,
1545,
1541,
1538,
1536,
1535,
1536,
1541,
1548,
1557,
1566,
1572,
1571,
1565,
1557,
1552,
1555,
1565,
1577,
1587,
1590,
1587,
1580,
1571,
1561,
1549,
1536,
1525,
1519,
1521,
1532,
1547,
1560,
1566,
1563,
1552,
1540,
1532,
1531,
1535,
1539,
1540,
1535,
1527,
1522,
1525,
1535,
1548,
1559,
1564,
1561,
1556,
1551,
1549,
1552,
1557,
1563,
1567,
1569,
1569,
1570,
1571,
1571,
1568,
1562,
1554,
1547,
1544,
1545,
1548,
1551,
1551,
1550,
1550,
1552,
1555,
1557,
1556,
1553,
1549,
1547,
1547,
1548,
1548,
1546,
1543,
1542,
1543,
1546,
1549,
1551,
1552,
1554,
1558,
1565,
1575,
1585,
1590,
1589,
1582,
1572,
1563,
1557,
1554,
1551,
1547,
1539,
1530,
1523,
1523,
1529,
1540,
1551,
1557,
1557,
1553,
1548,
1545,
1546,
1551,
1557,
1562,
1565,
1564,
1561,
1558,
1555,
1552,
1548,
1542,
1534,
1527,
1523,
1526,
1536,
1553,
1570,
1582,
1587,
1585,
1581,
1577,
1575,
1573,
1569,
1561,
1552,
1545,
1542,
1545,
1550,
1556,
1559,
1560,
1559,
1558,
1558,
1557,
1557,
1555,
1554,
1554,
1555,
1557,
1559,
1560,
1560,
1559,
1558,
1558,
1559,
1560,
1561,
1559,
1556,
1555,
1557,
1564,
1575,
1583,
1586,
1580,
1569,
1557,
1549,
1546,
1546,
1546,
1547,
1547,
1549,
1552,
1555,
1554,
1549,
1541,
1535,
1533,
1537,
1543,
1548,
1550,
1549,
1546,
1545,
1548,
1554,
1562,
1567,
1568,
1564,
1555,
1547,
1544,
1547,
1556,
1569,
1581,
1589,
1590,
1586,
1577,
1566,
1556,
1547,
1543,
1544,
1548,
1554,
1558,
1561,
1562,
1560,
1554,
1544,
1533,
1523,
1519,
1522,
1531,
1541,
1548,
1551,
1552,
1553,
1555,
1557,
1557,
1553,
1545,
1537,
1532,
1532,
1537,
1544,
1551,
1557,
1561,
1564,
1565,
1566,
1565,
1563,
1561,
1558,
1555,
1553,
1553,
1554,
1556,
1558,
1558,
1555,
1551,
1548,
1546,
1546,
1548,
1551,
1555,
1559,
1562,
1564,
1561,
1555,
1548,
1544,
1544,
1548,
1555,
1558,
1558,
1554,
1552,
1554,
1562,
1573,
1583,
1588,
1583,
1572,
1557,
1546,
1541,
1543,
1548,
1551,
1550,
1546,
1542,
1541,
1542,
0,
2,
0,
2

};

#endif
