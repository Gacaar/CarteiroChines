import json
from pathlib import Path

def importJson(file):
    start = ''
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
            if node['isAcceptState'] == True:
                start = node['text']

    for link in links:
        if link['type'] == 'Link':
            nodeOrig = nodes[link['nodeA']]['text']
            nodeDest = nodes[link['nodeB']]['text']
            graph[nodeOrig].append(nodeDest)

    return graph, start