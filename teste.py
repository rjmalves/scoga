# Imports gerais de bibliotecas padrão
import pickle
from os import listdir
from os.path import isfile, join
from typing import Dict
from pandas import DataFrame
import numpy as np
import plotly.express as px


def process_lane_data() -> Dict[str, DataFrame]:
    """
    Processa os dados de arestas existentes no diretório da simulação.
    """
    result_dir = 'results/cross/1589325460'
    lane_data = DataFrame()
    # Para cada arquivo no diretório
    for f in listdir(result_dir):
        full_path = join(result_dir, f)
        if isfile(full_path):
            # Se o arquivo é de aresta, processa os dados
            if f == "lanes.pickle":
                with open(full_path, "rb") as fileref:
                    lane_data = pickle.load(fileref)
    # Retorna os dados de arestas
    return lane_data


def process_lane_id_figure(df: DataFrame):
    """
    """
    fig = px.area(df,
                  x='sampling_time',
                  y='average_speed',
                  labels={'sampling_time': '',
                          'average_speed': ''},
                  template='none',
                  facet_col='lane_id',
                  facet_col_wrap=3
                  )
    fig.update_traces(line={'width': 2})
    fig.update_yaxes({'showticklabels': False,
                        'showgrid': False})
    fig.update_xaxes({'zeroline': False,
                        'showticklabels': False,
                        'showgrid': False})
    return fig
    # fig.write_image("results/cross/1589312585/trafficlights.pdf")


if __name__ == "__main__":
    lane_data = process_lane_data()
    print(lane_data)
    fig = process_lane_id_figure(lane_data)
    fig.show()
