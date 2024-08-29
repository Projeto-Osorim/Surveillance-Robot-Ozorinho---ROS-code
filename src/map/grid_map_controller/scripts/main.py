import math
import time
from copy import copy
from typing import List, Dict
import rospy
from std_msgs.msg import String

serial_command_pub = rospy.Publisher("/serial_command", String, queue_size=10)

class Movimento:
    
    @staticmethod
    def frente():
        serial_command_pub.publish('W')
        rospy.loginfo('W')
        time.sleep(1.8)

    @staticmethod
    def virar90direita():
        serial_command_pub.publish('D')
        rospy.loginfo('D')
        time.sleep(1.2)

    @staticmethod
    def virar90esquerda():
        serial_command_pub.publish('A')
        rospy.loginfo('A')
        time.sleep(1.15)

    @staticmethod
    def virar180():
        serial_command_pub.publish('D')
        time.sleep(2.4)
        rospy.loginfo('S')

    @staticmethod
    def parar():
        serial_command_pub.publish('P')
        rospy.loginfo('P')

class Nodes:
    def __init__(self, x: int, y: int):
        self.x: int = x
        self.y: int = y
        self.f: float = 0.0
        self.g: float = 0.0
        self.h: float = 0.0
        self.vizinhos: List['Nodes'] = []

    def heuristica(self, objetivo: 'Nodes') -> float:
        return math.sqrt((self.x - objetivo.x) ** 2 + (self.y - objetivo.y) ** 2)

class AlgoritmoAStar:
    @staticmethod
    def a_estrela(inicio: Nodes, objetivo: Nodes, matriz: 'List[List[int]]') -> List[Nodes]:
        abertos: List[Nodes] = []
        veio_de: Dict[Nodes, Nodes] = {}
        custo_g: Dict[Nodes, float] = {}

        abertos.append(inicio)
        custo_g[inicio] = 0.0
        inicio.h = inicio.heuristica(objetivo)
        inicio.f = inicio.h

        while abertos:
            atual: Nodes = min(abertos, key=lambda no: no.f)
            abertos.remove(atual)

            if atual == objetivo:
                return AlgoritmoAStar.reconstruir_caminho(veio_de, inicio, objetivo)

            for vizinho in atual.vizinhos:
                if vizinho and matriz[vizinho.x][vizinho.y] == 0:
                    custo_g_tentativo: float = custo_g.get(atual, math.inf) + 1

                    if custo_g_tentativo < custo_g.get(vizinho, math.inf):
                        veio_de[vizinho] = atual
                        custo_g[vizinho] = custo_g_tentativo
                        vizinho.h = vizinho.heuristica(objetivo)
                        vizinho.f = vizinho.h + custo_g_tentativo
                        if vizinho not in abertos:
                            abertos.append(vizinho)
        return []

    @staticmethod
    def reconstruir_caminho(veio_de: Dict[Nodes, Nodes], inicio: Nodes, objetivo: Nodes) -> List[Nodes]:
        caminho: List[Nodes] = []
        atual: Nodes = objetivo
        while atual != inicio:
            caminho.append(atual)
            atual = veio_de.get(atual, inicio) 
        caminho.append(inicio)
        caminho.reverse()
        return caminho
    
def conexoesMatriz(linhas: int, colunas: int, nos: 'List[List[Nodes | None]]'):
    for i in range(linhas):  # y
        for j in range(colunas):  # x
            if nos[i][j]:
                # Corrigindo as direções com base na nova definição de x e y
                if i > 0 and nos[i - 1][j]:  # Acima
                    nos[i][j].vizinhos.append(nos[i - 1][j])
                if i < linhas - 1 and nos[i + 1][j]:  # Abaixo
                    nos[i][j].vizinhos.append(nos[i + 1][j])
                if j > 0 and nos[i][j - 1]:  # Esquerda
                    nos[i][j].vizinhos.append(nos[i][j - 1])
                if j < colunas - 1: 
                    if nos[i][j + 1]:  # Direita
                        nos[i][j].vizinhos.append(nos[i][j + 1])

def atualizar_orientacao(orientacao_atual, dx, dy):
    direcoes = ['N', 'L', 'S', 'O']  # Norte, Leste, Sul, Oeste
    if dy > 0:  # Movendo-se para leste
        nova_orientacao = 'L'
    elif dy < 0:  # Movendo-se para oeste
        nova_orientacao = 'O'
    elif dx < 0:  # Movendo-se para norte
        nova_orientacao = 'N'
    elif dx > 0:  # Movendo-se para sul
        nova_orientacao = 'S'
    else:
        return orientacao_atual  # Sem movimento, mantém a orientação

    # Calcula a rotação necessária
    rotacao = direcoes.index(nova_orientacao) - direcoes.index(orientacao_atual)
    if rotacao == 1 or rotacao == -3:
        time.sleep(1.26)
        Movimento.virar90direita()
        print("Virar para a direita")
    elif rotacao == -1 or rotacao == 3:
        time.sleep(1.26)
        Movimento.virar90esquerda()
        print("Virar para a esquerda")
    elif abs(rotacao) == 2:
        time.sleep(1.26)
        Movimento.virar180()
        print("Virar para trás")
    return nova_orientacao

def construirCaminho(caminho: List[Nodes]):
    # anterior = None
    # atual = None
    # proximo = None
    orientacao = 'N'
    quadranteAtual = copy(caminho[0])
    del caminho[0]
    # print(f"{quadranteAtual.x},{quadranteAtual.y}")
    for quadrante in caminho:
        #Movimentacao pelo eixo y
        dy=quadrante.y-quadranteAtual.y
        dx=quadrante.x-quadranteAtual.x
        orientacao=atualizar_orientacao(orientacao,dx,dy)
        print(f"dx={dx}, dy={dy}")
        print(f"{quadrante.x},{quadrante.y}")
        quadranteAtual=quadrante
        Movimento.frente()
        Movimento.parar()

def main():
    time.sleep(5)
    rospy.init_node("controlador")

    #8x13
    matriz3 = [
        [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
    ]

    matriz2 = [
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1],
        [1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1],
        [1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1],
        [1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1],
        [1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1],
    ]

    matriz1 = [
        [0,0,1,1],
        [0,0,1,0],
        [0,0,0,0],
        [0,1,0,0],
    ]

    matriz = matriz1

    # Criando nós a partir da matriz
    linhas = len(matriz)
    colunas = len(matriz[0])
    print(linhas)
    print(colunas)
    nos = [[Nodes(i, j) if matriz[i][j] == 0 else None for j in range(colunas)] for i in range(linhas)]

    # Estabelecendo conexões entre os nós
    conexoesMatriz(linhas, colunas, nos)

    inicio_x = 3
    inicio_y = 3
    objetivo_x = 0
    objetivo_y = 0

    inicio = nos[inicio_y][inicio_x]
    objetivo = nos[objetivo_y][objetivo_x]

    caminho = AlgoritmoAStar.a_estrela(inicio, objetivo, matriz)
    # Movimento.virar90direita()
    # Movimento.parar()
    # Movimento.virar90esquerda()
    # Movimento.parar()

    if caminho:
        print("Caminho encontrado:", [(no.x, no.y) for no in caminho])
        construirCaminho(caminho)
        #print(matriz)
    else:
        print("Caminho não encontrado.")

if __name__ == '__main__':
    main()