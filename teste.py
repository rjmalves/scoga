# Imports gerais de bibliotecas padrão
import pickle
from os import listdir
from os.path import isfile, join
from typing import Dict
from pandas import DataFrame
import plotly.express as px


def process_tl_data() -> Dict[str, DataFrame]:
    """
    Processa os dados de arestas existentes no diretório da simulação.
    """
    result_dir = 'results/cross/1589312585'
    tl_data = DataFrame()
    # Para cada arquivo no diretório
    for f in listdir(result_dir):
        full_path = join(result_dir, f)
        if isfile(full_path):
            # Se o arquivo é de aresta, processa os dados
            if f == "trafficlights.pickle":
                with open(full_path, "rb") as fileref:
                    tl_data = pickle.load(fileref)
    # Retorna os dados de arestas
    return tl_data


if __name__ == "__main__":
    tl_data = process_tl_data()
    tl_data['y'] = 1.0
    is_01 = tl_data['tl_id'] == '0-0'
    df = tl_data[is_01]
    fig = px.area(df,
                  x='sampling_time',
                  y='y',
                  color='state',
                  color_discrete_sequence=['#EF350D',
                                           '#0DEF85',
                                           '#F4CF38'],
                  title='Grupo semafórico {}'.format('0-0'),
                  labels={'sampling_time': 'Tempo (s)',
                          'y': '',
                          'state': 'Estado'},
                  template='none',
                  range_y=[0, 1]
                  )
    fig.update_traces(line={'width': 0})
    fig.update_layout(
        yaxis={'showticklabels': False,
               'showgrid': False,
               'zeroline': False},
        xaxis={'showgrid': False,
               'zeroline': False}
    )
    fig.write_image("results/cross/1589312585/trafficlights.pdf")
