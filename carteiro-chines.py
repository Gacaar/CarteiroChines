from queue import Queue
import numpy as np
from scipy.optimize import linear_sum_assignment
import random
from loadJson import importJson
from pathlib import Path

# Pasta onde os arquivos JSON estao
dataFolder = Path("C:/Users/castr/Dropbox/PIBIC_Gabriel_Douglas/Máquina de Estados/Circuitos Sequenciais/Modelos JSON/")

# Nome do arquivo JSON que representa o grafo a ser processado agora
fileName = 'paridade3_sem_texto.json'

# Define o formato do no, onde 'i' significa entrada, 'o' significa saida, e '*' significa um caractere que pode ser ignorado
nodeFormat = 'ii****o'

# Importa o grafo
file = dataFolder / fileName
graph, start = importJson(file)

# Define o no inicial caso nao tenha sido passado via JSON
if start == '':
    start = '00|00|0'


def ChinesePostman():
    RemoveIsolatedNodes()
    if(GraphSolvable()):
        imbalacedNodes = FindImbalacedNodes()
        bipartiteGraph, pairsPaths = FindShortestPaths(imbalacedNodes)
        optimalPaths = FindOptimalPairs(bipartiteGraph, pairsPaths)
        newGraph = InsertAditionalPaths(optimalPaths)
        eulerTour = Hierholzer(newGraph)
        printEulerTour(eulerTour)
        convertToSequence(eulerTour)
    else:
        print('Sem solução')


def importJson(file):
    f = open(file, 'r')
    str = f.read()
    str.replace('true','True')
    str.replace('false','False')
    graphJson = json.loads(str)
    nodes = graphJson['nodes']
    links = graphJson['links']

    graph = {}
    for node in nodes:
        if not node['textOnly']:
            graph[node['text']] = []

    for link in links:
        if link['type'] == 'Link':
            nodeOrig = nodes[link['nodeA']]['text']
            nodeDest = nodes[link['nodeB']]['text']
            graph[nodeOrig].append(nodeDest)

    return graph

def RemoveIsolatedNodes():
    return graph;

def GraphSolvable():
    return True;

# Estrutura 'nome do no': (out - in)
def FindImbalacedNodes():
    # inicializa o dicionario de balanceamento
    balance = {}
    for node in graph:
        balance[node] = 0;
    # contabiliza o balanco de cada no
    for originNode in graph:
        for destNode in graph[originNode]:
            balance[originNode] = balance[originNode] + 1;
            balance[destNode] = balance[destNode] - 1;
    # Mantem apenas os nos imbalanceados
    imbalaced = {node:balanceValue for node,balanceValue in balance.items() if balanceValue!=0}

    # Verifica se existe algum com grau maior que 9 (nao suportado)
    for node in imbalaced:
        if imbalaced[node] > 9:
            print("erro na execução, grau de entradas-saidas maior que 9")
            return

    return imbalaced

# Retorna o menor caminho entre todos os pares de nos imbalaceados (combinando os positivos com negativos). Lembrar de considerar cada no d vezes, onde d = |out-in|
# Lembrar de manter o caminho do grafo original 'anotado'
def FindShortestPaths(imbalacedNodes):
    positiveNodes = [];
    negativeNodes = [];

    # Cria as listas de nos imbalanceados positiva ou negativamente, repetindo d vezes
    for node in imbalacedNodes:
        d = imbalacedNodes[node]
        if d > 0:
            for i in range(d):
                positiveNodes += [node+str(i)]
        else:
            for i in range(abs(d)):
                negativeNodes += [node+str(i)]
    
    # Calcula os menores caminhos entre todos os pares 
    dist, optimalPaths = FloydWarshal()

    # Retorna o grafo bipartido e os caminhos otimos
    bipartiteGraph = {}
    pairsPaths = {}
    for nodeN in negativeNodes:
        bipartiteGraph[nodeN] = {}
        for nodeP in positiveNodes:
            pairsPaths[(nodeN[:-1], nodeP[:-1])] = optimalPaths[(nodeN[:-1], nodeP[:-1])]
            bipartiteGraph[nodeN][nodeP] = dist[nodeN[:-1]][nodeP[:-1]]

    return bipartiteGraph, pairsPaths


    
# Floy-Warshal algorithm
def FloydWarshal():
    V = len(graph) # Number of vertices V

    # Inicializa 'matriz' de distancias (dist[i,j] eh a distancia entre os nos i e j)
    # Inicializa 'matriz' de caminhos otimos
    dist = {}
    optimalPaths = {}
    for i in graph:
        dist[i] = {}
        for j in graph:
            optimalPaths[(i,j)] = [i,j]
            dist[i][j] = 0 if i==j else np.inf

    # Preenche a distancia dos nos vizinhos
    for node in graph:
        for destNode in graph[node]:
            dist[node][destNode] = 1

    for k in graph:
        for i in graph:
            for j in graph:
                if dist[i][j] > dist [i][k] + dist[k][j]:
                        dist[i][j] = dist [i][k] + dist[k][j]
                        optimalPaths[(i,j)] = [i] + optimalPaths[(i,k)][1:-1] + [k] + optimalPaths[(k,j)][1:-1] + [j]

    return dist, optimalPaths

def FindOptimalPairs(bipartiteGraph, paths):
    optimalPairs = HungarianMethod(bipartiteGraph)
    optimalPaths = {}
    for pair in optimalPairs:
        optimalPaths[pair] = paths[(pair[0][:-1], pair[1][:-1])]

    return optimalPaths
    

def HungarianMethod(bipartiteGraph):
    cost = np.empty((len(bipartiteGraph), len(bipartiteGraph)))
    graphCostBind = {} # relaciona o dicionario com a matriz de custos, uma vez que o dicionario nao tem ordem
    i = -1
    for node in bipartiteGraph:
        i += 1
        j = -1
        for weight in bipartiteGraph[node]:
            j += 1
            cost[i][j] = bipartiteGraph[node][weight]
            graphCostBind[(i,j)] = (node,weight)


    row_ind, col_ind = linear_sum_assignment(cost)

    optimalPairs = []
    for i in row_ind:
        j = col_ind[i]
        optimalPairs.append(graphCostBind[(i,j)])

    return optimalPairs


def InsertAditionalPaths(optimalPaths):
    newGraph = graph

    for pair in optimalPaths:
        actualNode = ''
        for step in optimalPaths[pair]:
            if actualNode != '':
                newGraph[actualNode].append(step)
            actualNode = step

    return newGraph
    

def Hierholzer(newGraph):
    nodes = list(newGraph.keys())
    if start != '':
        startNode = start
    else:
        startNode = nodes[0]
    currentNode = ''
    subtour = []
    tour = []
    eulerianPath = False

    visitedEdges = {}
    unvisitedEdges = {}
    for node in newGraph:
        visitedEdges[node] = ([])
        unvisitedEdges[node] = (newGraph[node])
    
    while not eulerianPath:
        currentNode = startNode
        subtour.append(startNode)
        while True:
            # Escolhe um caminho nao visitado a partir do no inicial do subtour
            nextNode = random.choice(unvisitedEdges[currentNode])
            unvisitedEdges[currentNode].remove(nextNode)
            visitedEdges[currentNode].append(nextNode)
            currentNode = nextNode
            subtour.append(nextNode)
            # Verifica se o subtour esta completo
            if currentNode == startNode:
                break
        
        # Incorpora subtour no tour
        if len(tour) != 0:
            for i in range(len(tour)):
                if tour[i] == subtour[0]:
                    tour[i:i+1] = subtour
                    break
        else:
            tour += subtour
        subtour = []

        # Checa se todos os caminhos foram tomados, se nao, inicia um novo subtour
        startNode = ''
        for node in tour:
            if len(unvisitedEdges[node]) != 0:
                startNode = node
                break
        if startNode == '':
            eulerianPath = True

    return tour
            
        

def printEulerTour(eulerTour):
    print('// Numero minimo de transicoes: ' + str(len(eulerTour)-1))
    print('// ' + str(eulerTour))


def convertToSequence(eulerTour):

    # Define uma funcao que busca um char em uma string e retorna todos os indices que ela foi encontrada
    def find(s, ch):
        return [i for i, ltr in enumerate(s) if ltr == ch]

    # Busca os indices que contem as entradas e saidas
    inputIndex = find(nodeFormat, 'i')
    outputIndex = find(nodeFormat, 'o')

    # Identifica o numero de entradas e saidas
    nIn  = len(inputIndex)
    nOut = len(outputIndex)

    seq = ""
    for node in eulerTour:
        # Para cada no, define quais os bits de entrada e saida
        nodeInput = ''
        nodeOutput = ''

        for index in inputIndex:
            nodeInput += node[index]

        for index in outputIndex:
            nodeOutput += node[index]

        seq += 'std::make_tuple("'
        # Entradas
        seq += '0'*(8-nIn) + nodeInput + '","'
        # Saidas
        seq += '0'*(8-nOut) + nodeOutput + '","'
        # Saidas validas
        seq += '0'*(8-nOut) + '1'*nOut + '"), // ' + str(node) + '\n'

    fillLines = 4 - len(eulerTour) % 4
    for i in range(fillLines):
        seq += 'std::make_tuple("00000000","00000000","00000000"), // FILL\n'
    
    # remove a ultima virgula
    seq = ' '.join(seq.rsplit(',', 1))

    print(seq)

if __name__ == '__main__':
    ChinesePostman();