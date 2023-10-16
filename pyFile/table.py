'''
@ name: Austin
@ description: 使用python实现直接在控制台输出一个格式化表格
@ Date: 2023-10-08 13:32:11
@ LastEditDate: 
'''

maxCellLen = []
# 记录每个单元格包含的中文字符
CellLen_CH = []
maxRowLen = 0
table = []

def countWidth(str):
    '''判断单元格中包含多少个中文字符

    Args:
        str (string): 单元格内容的字符串形式

    Returns:
        width: 单元格内容宽度，中文字符占两个，所以用len(str)+hasCh
        hasCh: 单元格内包含的中文字符个数
    '''
    hasCh = 0

    for i in range(0, len(str)):
        if '\u4e00' <= str[i] <= '\u9fff':
            hasCh = hasCh + 1
    
    return len(str)+hasCh, hasCh

def maxLen(list):
    '''输入一个list，更新maxCellLen和maxRowLen
    \n 同时记录其中有多少中文符号，在后续的输出会用到

    Args:
        list (list): 新输入的列表
    '''

    # 这是为了声明这两个变量使用的是全局变量
    global maxRowLen
    global maxCellLen
    global CellLen_CH

    maxRowLen = maxRowLen if maxRowLen > len(list) else len(list)  
    lenList = []

    for i in range(0, len(list)):
        # 单元格元素可能是数字，无法使用len()，所以需要转换为字符串
        temp = str(list[i])
        width, hasCh = countWidth(temp)
        lenList.append(hasCh)

        # 如果i>=len(), 说明maxCellLen这个list还有这一列
        if i >= len(maxCellLen):
            maxCellLen.append(width)
        else:
            maxCellLen[i] = maxCellLen[i] if maxCellLen[i] > width else width 
    
    CellLen_CH.append(lenList)

def addRow(list_row):
    '''向table中添加行(列表)
    \n确定最多行元素是多少，最大单元格长度是多少

    Args:
        list_row (list): table中的行元素
    '''
    table.append(list_row)
    maxLen(list_row) 

def addtable(input_table):
    '''直接输入一个二维数组作为table

    Args:
        input_table (array[][]): 二维数组
    '''
    for i in range(0, len(input_table)):
        table.append(input_table[i])
        maxLen(table[i])

def addTitles(list_title):
    '''输入table的头行

    Args:
        list_title (list): title
    '''
    table.append(list_title)
    maxLen(list_title)

def printTableHeadAndFoot():
    '''输出table的第一行和最后一行符号
    '''
    for i in range(0, maxRowLen):
        print("+", end="")
        print('-' * (maxCellLen[i]+2), end="") 
    print('+')  

def showTable():
    '''输出table，关键之处在于，输出时要用最大长度-其中包含的中文字符
    \n因为中文字符占据2个长度
    '''
    printTableHeadAndFoot()

    for i in range(0, len(table)):
        for j in range(0, len(table[i])):
            print('| {:^{}} '.format(str(table[i][j]), maxCellLen[j]-CellLen_CH[i][j]), end="")
        print('|')
        
        if i == 0 and len(table) != 1:
            printTableHeadAndFoot()

    printTableHeadAndFoot()
    print(maxCellLen)
