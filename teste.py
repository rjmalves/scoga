# Imports gerais de bibliotecas padrão
import pickle
from os import listdir
from os.path import isfile, join
from typing import Dict
from pandas import DataFrame
import plotly.express as px


def process_node_data() -> Dict[str, DataFrame]:
    """
    Processa os dados de arestas existentes no diretório da simulação.
    """
    result_dir = 'results/cross/1589416083'
    node_data = DataFrame()
    # Para cada arquivo no diretório
    for f in listdir(result_dir):
        full_path = join(result_dir, f)
        if isfile(full_path):
            # Se o arquivo é de aresta, processa os dados
            if f == "nodes.pickle":
                with open(full_path, "rb") as fileref:
                    node_data = pickle.load(fileref)
    # Adiciona a coluna 'y' = 1.0
    node_data['y'] = 1.0
    # Retorna os dados de arestas
    return node_data


def process_node_id_figure(df: DataFrame):
    """
    """
    fig = px.area(df,
                    x='sampling_time',  # instantes de tempo
                    y='y',  # 1.0 constante só para ter a área
                    color='stages',  # cor segundo o estado
                    color_discrete_sequence=['#588da8',  # E1
                                             '#ccafaf',  # E2
                                             '#e58a8a'],  # E3
                    labels={'sampling_time': '',
                            'y': '',
                            'stages': 'Estágio'},  # substitui os nomes
                    template='none',  # template minimalista
                    range_y=[0, 1],  # deixa o eixo y preenchido (até 1.0)
                    facet_col='node_id',  # gera subplots segundo o semáforo
                    facet_col_wrap=1,  # salta de linha após 1 na coluna
                    title='',  # título geral
                    width=640,  # dimensões da imagem
                    height=480  # dimensões da imagem
                    )
    fig.update_traces(line={'width': 0})
    fig.update_yaxes({'showticklabels': False,
                        'showgrid': False})
    fig.update_xaxes({'zeroline': False,
                      'showticklabels': False,
                        'showgrid': False})
    return fig
    # fig.write_image("results/cross/1589312585/trafficlights.pdf")


if __name__ == "__main__":
    node_data = process_node_data()
    print(node_data)
    fig = process_node_id_figure(node_data)
    fig.show()
